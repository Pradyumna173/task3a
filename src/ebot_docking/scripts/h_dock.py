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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from payload_service.srv import PayloadSW
from my_robot_interfaces.srv import DockSw
from std_msgs.msg import Bool
from functools import partial
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32, Float32MultiArray


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

        self.subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.subscriber = self.create_subscription(
            Float32, "/orientation", self.orientation_sub_callback, 10
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

        self.ultra_sub = self.create_subscription(
            Float32MultiArray, "ultrasonic_sensor_std_float", self.ultra_callback, 10
        )

        self.vel_pub.publish(Twist())

        self.dock_entry_y = None
        self.dock_entry_x = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.ultra_left = 0.0
        self.ultra_right = 0.0
        self.box_on_bot = None
        self.yaw = None
        self.ultra_left = None
        self.ultra_right = None

        self.processing_dock = False
        self.payload_dropped = False
        self.payload_called = False

        self.current_activity = None

        self.dock_dict = {}
        self.dock_dict["con2"] = [2.35, 3.47]
        self.dock_dict["rec"] = [0.4, -2.42]
        self.dock_dict["con1"] = [-4.55, 3.36]

    # callback for ultrasonic subscription
    def ultra_callback(self, msg):
        self.ultra_left = msg.data[4]
        self.ultra_right = msg.data[5]

    def orientation_sub_callback(self, msg):
        self.current_theta = msg.data
        print(self.yaw)

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
        if self.target == "rec":
            self.current_activity = "move"
        else:
            self.current_activity = "move"
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
            if act == "move":
                distance_to_goal = -math.sqrt(
                    (self.dock_entry_x - self.current_x) ** 2
                    + (self.dock_entry_y - self.current_y) ** 2
                )

                if abs(distance_to_goal) > 0.05:
                    angle_to_goal = math.atan2(
                        self.dock_entry_y - self.current_y,
                        self.dock_entry_x - self.current_x,
                    )
                    angle_diff = self.normalize_angle(
                        angle_to_goal - (self.current_theta + math.pi)
                    )

                    if abs(angle_diff) > math.pi / 2:
                        # Reverse motion
                        angle_diff = self.normalize_angle(
                            angle_diff - math.pi
                            if angle_diff > 0
                            else angle_diff + math.pi
                        )
                        reverse = True
                    else:
                        reverse = False

                    if abs(angle_diff) > 0.1:
                        vel_msg.angular.z = 5.0 * angle_diff
                        vel_msg.linear.x = 0.0
                    else:
                        vel_msg.linear.x = (
                            -0.8 * distance_to_goal
                            if reverse
                            else 0.8 * distance_to_goal
                        )
                else:
                    vel_msg = Twist()
                    self.current_activity = "align"
            elif act == "align":
                if abs(self.current_theta + 1.57) > 0.02:
                    vel_msg.angular.z = 5.0 * (-1.57 - self.current_theta)
                    vel_msg.linear.x = 0.0
                else:
                    vel_msg = Twist()
                    self.current_activity = "rev"

                if vel_msg.angular.z > 2.0:
                    vel_msg.angular.z = 2.0
                elif vel_msg.angular.z < -2.0:
                    vel_msg.angular.z = -2.0
            elif act == "rev":
                if self.target == "rec":
                    stop_dist = 0.05
                else:
                    stop_dist = 0.27
                if self.ultra_left > stop_dist:
                    us_diff = self.ultra_left - self.ultra_right
                    vel_msg.angular.z = -8.0 * us_diff
                    if abs(us_diff) < 0.03:
                        vel_msg.linear.x = -1.5 * self.ultra_left
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
                self.call_payload(False)  # True is receive, False is Drop
            self.vel_pub.publish(vel_msg)
            time.sleep(0.03)

            if self.payload_dropped:
                vel_msg.linear.x = 0.5
                self.vel_pub.publish(vel_msg)
                self.processing_dock = False

    def odom_callback(self, msg):
        # Update current position and orientation from odometry data
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        # _, _, self.current_theta = euler_from_quaternion(
        #     [orientation.x, orientation.y, orientation.z, orientation.w]
        # )

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
