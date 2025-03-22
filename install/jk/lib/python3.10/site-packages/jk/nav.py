##!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import sys
import time as tm
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from tf_transformations import quaternion_from_euler

from ebot_docking.srv import DockSw, PassingService
from usb_servo.srv import ServoSw

EBOT_ID = 6

CONTROLLER_RATE = 0.05
NAV_RATE = 1.0


class Nav(Node):
    def __init__(self):
        super().__init__("nav_node")

        self.waypoints = [
            [2.50, -2.8, -1.57],
            [2.45, 2.1, -2.95],
            [2.5, -1.2, -3.0],
        ]

        self.align_stops = {0: 20.0, 1: 69.0, 2: 32.0}
        self.get_out_dist = {0: 150.0, 1: 120.0, 2: 200.0}

        self.waypoint_index = 0

        # Ultrasonics
        # rear side of robot
        self.back_left = 0.0
        self.back_right = 0.0
        # front side of robot
        self.front_left = 0.0
        self.front_right = 0.0
        # Left side of robot
        self.left_front = 0.0
        self.left_back = 0.0

        # IMU
        self.yaw = 0.0
        self.nav_yaw = 0.0

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.ultra_sub = self.create_subscription(
            Float32MultiArray,
            "ultrasonic_sensor_std_float",
            self.callback_ultra_sub,
            10,
        )

        self.imu_sub = self.create_subscription(
            Float32, "/orientation", self.callback_imu_sub, 10
        )

        self.nav_timer = self.create_timer(NAV_RATE, self.nav_manager)

    def nav_manager(self):
        stamped_goal = self.create_pose_stamped(*self.waypoints[self.waypoint_index])

        self.nav.goToPose(stamped_goal)
        self.nav_timer.cancel()
        self.nav_timer = self.create_timer(NAV_RATE, self.check_nav)

    def check_nav(self):
        if self.nav.isTaskComplete():
            result = self.nav.getResult()

            if result == TaskResult.SUCCEEDED:
                self.nav_timer.cancel()

                self.nav_timer = self.create_timer(
                    CONTROLLER_RATE,
                    partial(self.align_perp, self.align_stops[self.waypoint_index]),
                )

    def align_perp(self, stop_at):
        if self.back_left == 0.0:
            self.get_logger().warn("Ultrasonic not working...")
            return

        self.get_logger().info("Aligning Perpendicular...")
        front_us_diff = self.front_left - self.front_right
        front_stop_dist = stop_at

        vel_msg = Twist()

        if abs(front_us_diff) > 3.0:
            vel_msg.angular.z = 0.01 * front_us_diff
            vel_msg.linear.x = 0.0
        elif (self.front_left > (stop_at + 2.5)) or (self.front_left < (stop_at - 2.5)):
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.03 * (self.front_left - front_stop_dist)
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.nav_yaw = self.yaw
            self.nav_timer.cancel()
            self.nav_timer = self.create_timer(CONTROLLER_RATE, self.align_yaw)

        self.vel_pub.publish(vel_msg)

    def align_yaw(self):
        self.get_logger().info("Aligning Yaw for Docking...")
        if self.waypoint_index == 0:
            goal_yaw = self.nav_yaw + (math.pi / 2.0)
        else:
            goal_yaw = self.nav_yaw - (math.pi / 2.0)

        if goal_yaw < 0.0:
            goal_yaw += 6.28
        else:
            goal_yaw = goal_yaw % 6.28

        vel_msg = Twist()

        if abs(goal_yaw - self.yaw) > 0.1:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = (goal_yaw - self.yaw) * 0.07
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.nav_timer.cancel()
            self.call_dock()

        self.vel_pub.publish(vel_msg)

    def call_dock(self):
        client = self.create_client(DockSw, "/docking")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Docking Server...")

        request = DockSw.Request()

        future = client.call_async(request)
        self.get_logger().info("\nDocking Called\n")
        future.add_done_callback(partial(self.callback_call_dock))

    def callback_call_dock(self, future):
        response = future.result()
        if response:
            self.get_logger().info("Docking Done")
            if self.waypoint_index == 0:
                self.call_passing_service()
            else:
                self.call_servo_toggle()

    def call_passing_service(self):
        client = self.create_client(PassingService, "/passing_service")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for passing service...")

        request = PassingService.Request()

        future = client.call_async(request)
        self.get_logger().info("\nPassing Service Called\n")
        future.add_done_callback(partial(self.callback_call_passing_service))

    def callback_call_passing_service(self, future):
        try:
            response = future.result()
            if response.success:
                tm.sleep(2)
                curr_index = self.waypoint_index
                self.nav_timer = self.create_timer(
                    CONTROLLER_RATE,
                    partial(self.get_out, self.get_out_dist[curr_index]),
                )
                self.waypoint_index = response.conveyer
                self.get_logger().info("\nPassing Service Done\n")

        except Exception:
            self.get_logger().error("Passing Service Call Failed!")

    def call_servo_toggle(self):
        client = self.create_client(ServoSw, "/toggle_usb_servo")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo_toggle service...")

        request = ServoSw.Request()
        request.servostate = True

        future = client.call_async(request)
        self.get_logger().info("\nServo Called\n")
        future.add_done_callback(partial(self.callback_call_servo_toggle))

    def callback_call_servo_toggle(self, future):
        try:
            response = future.result()
            tm.sleep(2)
            curr_index = self.waypoint_index
            self.nav_timer = self.create_timer(
                CONTROLLER_RATE, partial(self.get_out, self.get_out_dist[curr_index])
            )
            self.waypoint_index = 0
            self.get_logger().info("\nServo Toggle Done\n")
        except Exception:
            self.get_logger().error("Servo Toggle Service Failed")

    def get_out(self, out_dist):
        self.get_logger().info("Getting out...")
        vel_msg = Twist()
        if self.back_left < out_dist:
            vel_msg.linear.x = 0.3
        else:
            vel_msg.linear.x = 0.0
            self.nav_timer.cancel()
            self.nav_timer = self.create_timer(NAV_RATE, self.nav_manager)

        self.vel_pub.publish(vel_msg)

    def create_pose_stamped(self, x, y, yaw=0.0):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def callback_ultra_sub(self, msg):
        self.front_right = msg.data[0]
        self.front_left = msg.data[1]

        self.left_front = msg.data[2]
        self.left_back = msg.data[3]

        self.back_right = msg.data[4]
        self.back_left = msg.data[5]

    def callback_imu_sub(self, msg):
        self.yaw = msg.data


def main():
    rclpy.init(args=sys.argv)

    nav_node = Nav()

    executor = MultiThreadedExecutor(2)
    executor.add_node(nav_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
