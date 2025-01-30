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
#from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw
from std_msgs.msg import Bool
from functools import partial
from std_msgs.msg import Float32MultiArray


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
        self.ultra_sub = self.create_subscription(
            Float32MultiArray,
            "ultrasonic_sensor_std_float",
            self.ultra_callback,
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

        self.dock_entry_y = None
        self.dock_entry_x = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.curr_dist = 0.0
        self.comp_dist = 0.0
        self.box_on_bot = None

        self.processing_dock = False
        self.payload_dropped = False
        self.payload_called = False

        self.current_activity = None

        self.dock_dict = {}
        self.dock_dict["con2"] = [2.35, 3.47]
        self.dock_dict["rec"] = [0.4, -2.42]
        self.dock_dict["con1"] = [-4.55, 3.36]

    def docking_server_callback(self, request, response):

        self.get_logger().info("Request Received")
        print(request)

        self.payload_dropped = False
        self.payload_called = False

        self.dock_entry_x, self.dock_entry_y = self.dock_dict[request.target]
        self.target = request.target
        self.box_on_bot = request.box_number
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
                stop_dist = 22
                if self.curr_dist > stop_dist:
                    us_diff = self.curr_dist - self.comp_dist
                    vel_msg.angular.z = -0.2 * us_diff
                    if abs(us_diff) < 3:
                        vel_msg.linear.x = -0.5
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
            time.sleep(0.05)

    def ultra_callback(self, msg):
        self.curr_dist = msg.data[4]
        self.comp_dist = msg.data[5]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    dock_node = Docking()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(dock_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
