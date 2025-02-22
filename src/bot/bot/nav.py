#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy, math, time, sys
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ebot_docking.srv import DockSw, PassingService
from tf_transformations import quaternion_from_euler
from functools import partial
from geometry_msgs.msg import Twist
from usb_servo.srv import ServoSw


class Nav(Node):
    def __init__(self):
        super().__init__("ebot_nav")

        self.pose_dict = {
            "rec": [2.95, -2.7, 2.90],
            "con1": [2.8, 1.98, 3.14],
            "con2": [2.6, -1.2, 3.14],
        }

        self.activity_queue = ["rec"]

        self.box_on_bot = "nothing"
        self.docking_done = False

        self.first_time = True

        self.servo_angle = 0
        self.toggle_done = False

        self.client_group = MutuallyExclusiveCallbackGroup()
        self.nav_group = MutuallyExclusiveCallbackGroup()

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.nav_timer = self.create_timer(
            1.0, self.nav_manager, callback_group=self.nav_group
        )

    def nav_manager(self):
        if not self.activity_queue:
            forward_msg = Twist()
            self.vel_pub.publish(forward_msg)
            time.sleep(0.03)
            return

        goal = self.create_pose_stamped(*self.pose_dict[self.activity_queue[0]])
        if self.go_to_goal(goal):

            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            self.call_dock(self.activity_queue[0])

            while not self.docking_done:
                pass

            self.docking_done = False

            if self.activity_queue[0] == "rec":
                if not self.first_time:
                    self.call_servo_toggle(True)
                    
                    while not self.toggle_done:
                        pass
                    self.toggle_done = False
                self.first_time = False

                self.call_passing_service()
                while self.box_on_bot == "nothing":
                    pass
            else:
                self.call_servo_toggle(True)
                while not self.toggle_done:
                    pass
                self.toggle_done = False
                self.box_on_bot = "nothing"

            time.sleep(2.0)

            forward_msg = Twist()
            forward_msg.linear.x = 0.5
            self.vel_pub.publish(forward_msg)

            self.activity_queue.pop(0)

    def create_pose_stamped(self, x, y, yaw=0.0):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def go_to_goal(self, pose_stamped):
        self.nav.goToPose(pose_stamped)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal SUCCEEDED")
            return True
        else:
            self.get_logger().info("Goal FAILED")
            return False

    def call_servo_toggle(self, state):
        client = self.create_client(
            ServoSw, "/toggle_usb_servo", callback_group=self.client_group
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Servo Toggle Server...")

        request = ServoSw.Request()
        request.servostate = state

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_dock))

    def callback_call_servo_toggle(self, future):
        response = future.result()
        try:
            response = future.result()
            if response:
                self.toggle_done = True
                self.servo_angle = response.num
        except Exception as e:
            self.get_logger().error("Servo Toggle Failed")

    def call_passing_service(self):
        self.get_logger().info("Passing Service Called")
        client = self.create_client(
            PassingService, "/passing_service", callback_group=self.client_group
        )

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Passing Server...")

        self.box_on_bot = "nothing"
        request = PassingService.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_passing_service))

    def callback_call_passing_service(self, future):
        try:
            response = future.result()
            if response.success:
                box_number = int(response.box_number)
                self.box_on_bot = "box" + str(box_number)
                if box_number % 2 != 0:  # Check if the number is odd
                    self.activity_queue.append("con2")
                    self.activity_queue.append("rec")
                else:
                    self.activity_queue.append("con1")
                    self.activity_queue.append("rec")
        except Exception as e:
            self.get_logger().error("Passing Service Call Failed")

    def call_dock(self, target):
        self.get_logger().info("Docking Service Called")
        client = self.create_client(
            DockSw, "/docking", callback_group=self.client_group
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Docking Server...")

        request = DockSw.Request()
        request.target = target
        request.box_number = self.box_on_bot

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_dock, target=target))

    def callback_call_dock(self, future, target):
        try:
            response = future.result()
            self.docking_done = response.success
            self.get_logger().info("Docking Done")
        except Exception as e:
            self.get_logger().error("Docking Call Failed")


def main():
    rclpy.init(args=sys.argv)

    nav_node = Nav()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
