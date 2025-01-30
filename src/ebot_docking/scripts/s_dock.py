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

        self.ultrar_sub = self.create_subscription(
            Range, "ultrasonic_rr/scan", self.ultrarsub_callback, 10
        )

        self.ultral_sub = self.create_subscription(
            Range, "ultrasonic_rl/scan", self.ultralsub_callback, 10
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
        """
        Purpose:
        ---
        Handles docking requests by managing the docking process for ebot.

        Input Arguments:
        ---
        `request` : DockSw
            The request object containing details about the docking target.
            This contains name of conveyer where ebot is being docked

        `response` : DockSw
            Success status of the docking operation.

        Process:
        ---
        1. Retrieves the docking entry position (x, y coordinates) for the requested target from
        the `dock_dict`. A hashmap that contains the alignment points for each conveyer
        3. Sets up flags to manage the docking process:
            - `payload_dropped`: Indicates whether the payload has been dropped off.
            This is addressed in the payload callback
            - `payload_called`: Indicates whether the payload service has been called
            - `current_activity`: Indicates the robot's current activity.
        4. Sets `processing_dock` to `True` to begin the docking process, which continues until
        `processing_dock` is set to `False`
        5. Upon completion of docking, sets the `success` attribute of the response to `True`.

        Returns:
        ---
        `response` : [ CustomResponseType ]
            Success status of the docking operation.

        Example call:
        ---
        Called automatically as part of a ROS2 service callback when a docking client
        request is made.
        """

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
        """
        Purpose:
        ---
        Handles docking operations by executing the current activity (`move`, `align`, `rev`):
        - **move**: Navigates to the docking entry point, adjusts for distance and angle, and
        adjusts in reverse motion if needed.
        - **align**: Rotates the robot to align with the docking station.
        - **rev**: Reverses the robot to the docking position.
        Publishes velocity commands (`vel_msg`) to guide the robot and transitions activities
        based on progress. Ends docking when payload service is complete.

        Example call:
        ---
        Timer based call
        """

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

    def call_payload(self, action):  # true is receive, false is drop
        client = self.create_client(PayloadSW, "/payload_sw")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Payload Server...")

        request = PayloadSW.Request()
        request.receive = action
        request.drop = not action
        request.box_name = self.box_on_bot

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_payload, action=action))

    def callback_call_payload(self, future, action):
        try:
            response = future.result()
            self.get_logger().info(response.message)
            if response.message == "Payload Dropped Successfully !":
                self.payload_dropped = True
        except Exception as e:
            self.get_logger().error("Payload Service Call Failed")

    def ultrarsub_callback(self, msg):
        self.curr_dist = msg.range

    def ultralsub_callback(self, msg):
        self.comp_dist = msg.range

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
