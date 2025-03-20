#!/usr/bin/python3
# -*- coding: utf-8 -*-

from functools import partial

import globals
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

from usb_servo.srv import ServoSw


class Dock(Node):
    def __init__(self):
        super().__init__("dock_node")
        self.isFirstDock = False
        self.us_on = False

        if globals.EBOT_ID == 6:
            self.ultra_sub = self.create_subscription(
                Float32MultiArray,
                "ultrasonic_sensor_std_float",
                self.ultra_callback,
                10,
            )
        else:
            self.ultrar_sub = self.create_subscription(
                Range,
                "ultrasonic_rr/scan",
                self.ultrarsub_callback,
                10,
            )

            self.ultral_sub = self.create_subscription(
                Range,
                "ultrasonic_rl/scan",
                self.ultralsub_callback,
                10,
            )

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.dock_timer = None
        self.dock_check = self.create_timer(2.0, self.check_dock)

    def dock_control(self):
        if globals.EBOT_ID == 6:
            stop_dist = 28.0
        else:
            stop_dist = 15.0
        vel_msg = Twist()

        if (self.left_dist > 80.0) and (self.right_dist > 80.0):
            vel_msg.linear.x = -0.3
            vel_msg.angular.z = 0.0
        elif (self.left_dist > stop_dist) and (self.right_dist > stop_dist):
            range_diff = self.left_dist - self.right_dist
            vel_msg.angular.z = 0.03 * range_diff
            vel_msg.linear.x = 0.0
            if abs(range_diff) < 5.0:
                vel_msg.linear.x = -0.003 * ((self.left_dist + self.right_dist) / 2.0)
                vel_msg.angular.z = 0.0
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            if globals.EBOT_ID == 6:
                if self.isFirstDock:
                    self.call_servo_toggle(True)

            if self.dock_timer:
                self.dock_timer.destroy()

            self.isFirstDock = True
            globals.REACHED_POSE = False
            print("DOCKING DONE \n")

            self.dock_check = self.create_timer(2.0, self.check_dock)

        self.vel_pub.publish(vel_msg)

    def check_dock(self):
        if globals.REACHED_POSE and self.us_on:
            self.dock_timer = self.create_timer(0.05, self.dock_control)
            self.dock_check.destroy()

        if not self.us_on:
            self.get_logger().warn("Ultrasonic not working...")

    def call_servo_toggle(self, state):
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
            if self.dock_timer:
                self.dock_timer.destroy()

        except Exception as e:
            self.get_logger().error(f"Servo Toggle Failed: {e}")

    def ultra_callback(self, msg):
        self.left_dist = msg.data[5]
        self.right_dist = msg.data[4]
        self.us_on = True

    def ultrarsub_callback(self, msg):
        self.right_dist = msg.range * 100
        self.us_on = True

    def ultralsub_callback(self, msg):
        self.left_dist = msg.range * 100
        self.us_on = True
