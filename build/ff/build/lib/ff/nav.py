##!/usr/bin/python3
# -*- coding: utf-8 -*-


from math import sqrt

import globals
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


class Nav(Node):
    def __init__(self):
        super().__init__("nav_node")
        if globals.EBOT_ID == 6:
            self.waypoints = [
                [2.50, -2.827, 2.95],
                [2.45, 2.1, -2.95],
                [2.5, -1.2, -3.0],
            ]
        else:
            self.waypoints = [
                [0.4, -2.4, 3.14],
                [-4.0, 2.89, -1.57],
                [2.32, 2.55, -1.57],
            ]

        self.waypoint_index = 0
        self.x = 0.0
        self.y = 0.0

        self.saved_x = 0.0
        self.saved_y = 0.0

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.nav_timer = self.create_timer(1.0, self.nav_manager)
        self.check_timer = None
        self.box_check_timer = None
        self.vel_control_timer = None

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.callback_odom_sub, 10
        )

    def nav_manager(self):
        stamped_goal = self.create_pose_stamped(*self.waypoints[self.waypoint_index])

        self.nav.goToPose(stamped_goal)

        if self.nav_timer:
            self.nav_timer.destroy()
            self.check_timer = self.create_timer(1.0, self.check_nav)

    def check_nav(self):
        if self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            result = self.nav.getResult()

            if result == TaskResult.SUCCEEDED:
                globals.REACHED_POSE = True
                self.saved_x = self.x
                self.saved_y = self.y
                if self.check_timer:
                    self.check_timer.destroy()
                    self.box_check_timer = self.create_timer(1.0, self.check_box)

    def check_box(self):
        if not globals.REACHED_POSE:
            globals.BOX_REQUEST = 1  # Asks arm script to pass box
        print("Checking")
        if globals.BOX_REQUEST == 3:  # If box is dropped
            if self.box_check_timer:
                self.box_check_timer.destroy()
            print("BAAHER PAD BHAUUUU...\n")
            self.vel_control_timer = self.create_timer(0.05, self.vel_controller)

    def vel_controller(self):
        dist = sqrt(pow((self.saved_y - self.y), 2) + pow((self.saved_x - self.x), 2))

        vel_msg = Twist()

        vel_msg.linear.x = 0.03 * dist * 10.0
        if vel_msg.linear.x > 0.5:
            vel_msg.linear.x = 0.5

        if dist < 0.05:
            print("AALO BAHER\n")
            vel_msg.linear.x = 0.0
            pose_stamped = self.create_pose_stamped(
                *self.waypoints[self.waypoint_index]
            )
            self.nav.setInitialPose(pose_stamped)
            if self.vel_control_timer:
                self.vel_control_timer.destroy()
            if self.waypoint_index == 0:
                self.waypoint_index = globals.GOAL_CONVEYER
            else:
                self.waypoint_index = 0
            self.nav_timer = self.create_timer(1.0, self.nav_manager)
            globals.BOX_REQUEST = 4

        self.vel_pub.publish(vel_msg)

    def create_pose_stamped(self, x, y, yaw=0.0):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        # goal_pose.pose.orientation.x = 0.0#q_x
        # goal_pose.pose.orientation.y = 0.0#q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def callback_odom_sub(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
