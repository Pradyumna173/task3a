#!/usr/bin/env python3

"""
# Team ID:           LB_1048 
# Theme:             Logistic Cobot 
# Author List:       Pradyumna Ponkshe, Shantanu Ekad 
# Filename:          ebot_docking.py 
# Functions:         Comma separated list of functions in this file 
# Global variables:  List of global variables defined in this file, None if no global variables 
"""

import rclpy, math, time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from payload_service.srv import PayloadSW
from my_robot_interfaces.srv import DockSw
from std_msgs.msg import Bool
from functools import partial


class Docking(Node):
    def __init__(self):
        super().__init__("move_to_goal")

        self.docking_group = MutuallyExclusiveCallbackGroup()
        self.server = self.create_service(
            DockSw,
            "/docking",
            self.docking_server_callback,
            callback_group=self.docking_group,
        )
        self.sub_group = MutuallyExclusiveCallbackGroup()
        self.ultrar_sub = self.create_subscription(
            Range,
            "ultrasonic_rr/scan",
            self.ultrarsub_callback,
            10,
            callback_group=self.sub_group,
        )

        self.ultral_sub = self.create_subscription(
            Range,
            "ultrasonic_rl/scan",
            self.ultralsub_callback,
            10,
            callback_group=self.sub_group,
        )

        self.controller_group = MutuallyExclusiveCallbackGroup()
        self.controller_timer = self.create_timer(
            0.1, self.controller, callback_group=self.controller_group
        )
        self.vel_pub = self.create_publisher(
            Twist, "/cmd_vel", 10, callback_group=self.controller_group
        )

        self.vel_pub.publish(Twist())

        self.current_theta = 0.0
        self.curr_dist = 0.0
        self.comp_dist = 0.0
        self.box_on_bot = None

        self.processing_dock = False
        self.payload_dropped = False
        self.payload_called = False

        self.current_activity = None

    def docking_server_callback(self, request, response):

        self.get_logger().info("Request Received")
        print(request)

        self.current_activity = "rev"
        self.processing_dock = True

        while self.processing_dock:
            pass

        response.success = True
        return response

    def controller(self):
        if self.processing_dock:
            act = self.current_activity
            vel_msg = Twist()
            if act == "rev":
                stop_dist = 0.27
                if self.curr_dist > stop_dist:
                    us_diff = self.curr_dist - self.comp_dist
                    vel_msg.angular.z = -8.0 * us_diff
                    if abs(us_diff) < 0.03:
                        vel_msg.linear.x = -1.5 * self.curr_dist
                    else:
                        vel_msg.linear.x = 0.0
                else:
                    vel_msg = Twist()
                    self.current_activity = None
            else:
                vel_msg = Twist()
                if self.target == "rec":
                    self.vel_pub.publish(vel_msg)
                    self.processing_dock = False
                    return
                self.processing_dock = False
                
            self.vel_pub.publish(vel_msg)
            time.sleep(0.03)

    def ultrarsub_callback(self, msg):
        self.curr_dist = msg.range

    def ultralsub_callback(self, msg):
        self.comp_dist = msg.range


def main(args=None):
    rclpy.init(args=args)

    dock_node = Docking()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(dock_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
