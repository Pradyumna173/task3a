#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ebot_docking.srv import DockSw
from functools import partial
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range

from usb_servo.srv import ServoSw


class Docking(Node):
    def __init__(self):
        super().__init__("move_to_goal")

        self.process_dock = False
        self.toggle_done = 0
        self.toggle_ongoing = False
        self.current_activity = None

        self.us_on = False

        self.docking_group = MutuallyExclusiveCallbackGroup()

        self.server = self.create_service(
            DockSw,
            "/docking",
            self.docking_server_callback,
            callback_group=self.docking_group,
        )

        self.ultra_sub = self.create_subscription(
            Float32MultiArray,
            "ultrasonic_sensor_std_float",
            self.ultra_callback,
            10,
        )

        # self.ultrar_sub = self.create_subscription(
        #     Range,
        #     "ultrasonic_rr/scan",
        #     self.ultrarsub_callback,
        #     10,
        # )

        # self.ultral_sub = self.create_subscription(
        #     Range,
        #     "ultrasonic_rl/scan",
        #     self.ultralsub_callback,
        #     10,
        # )

        self.controller_group = MutuallyExclusiveCallbackGroup()

        self.controller_timer = self.create_timer(
            0.05, self.controller, callback_group=self.controller_group
        )
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def docking_server_callback(self, request, response):
        self.get_logger().info("Docking Request Received")

        while not self.us_on:
            self.get_logger().info("USonic Band")
            pass

        self.process_dock = True
        self.current_activity = request.target

        while self.process_dock:
            pass

        self.get_logger().info("Docking Done")
        response.success = True
        return response

    def controller(self):
        if not self.process_dock:
            return

        act = self.current_activity
        vel_msg = Twist()

        if act == "rec":
            if self.curr_dist > 70 and self.comp_dist > 70:
                vel_msg.linear.x = -0.3
                self.vel_pub.publish(vel_msg)
                self.get_logger().info("Maage ghetoy")
                return

        stop_dist = 35
        if (self.curr_dist > stop_dist) and (self.comp_dist > stop_dist):
            us_diff = self.curr_dist - self.comp_dist
            vel_msg.angular.z = -0.02 * us_diff
            if abs(us_diff) < 3:
                vel_msg.linear.x = -0.008 * self.curr_dist
                vel_msg.angular.z = 0.0
            else:
                vel_msg.linear.x = 0.0
        else:
            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            if act != "rec":
                if self.toggle_done != 2:
                    if not self.toggle_ongoing:
                        self.call_servo_toggle(True)

                    return

            self.process_dock = False
            self.current_activity = None
            self.toggle_done = 0

        self.vel_pub.publish(vel_msg)

    def ultra_callback(self, msg):
        self.curr_dist = msg.data[4]
        self.comp_dist = msg.data[5]
        self.us_on = True

    def ultrarsub_callback(self, msg):
        self.curr_dist = msg.range * 100
        self.us_on = True

    def ultralsub_callback(self, msg):
        self.comp_dist = msg.range * 100
        self.us_on = True

    def call_servo_toggle(self, state):
        self.toggle_ongoing = True

        client = self.create_client(ServoSw, "/toggle_usb_servo")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Servo Toggle Server...")

        request = ServoSw.Request()
        request.servostate = state

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_servo_toggle))

    def callback_call_servo_toggle(self, future):
        try:
            response = future.result()
            self.toggle_done += 1
            self.servo_angle = response.num
            self.toggle_ongoing = False
        except Exception as e:
            self.get_logger().error(f"Servo Toggle Failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    dock_node = Docking()

    executor = MultiThreadedExecutor()
    executor.add_node(dock_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
