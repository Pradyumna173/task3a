#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
* Team Id : LB#1048
* Author List : Pradyumna Ponkshe
* Filename: arm.py
* Theme: Logistic Cobot
* Functions: Arm{passingServerCB, pick_box, place_box, call_attach_box, callback_call_attach_box,
    call_detach_box, callback_call_detach_box, process_image, servo_lookup, find_transform, broadcast_transform, detect_aruco, rotation_matrix_to_euler_angles, call_servo_trigger, callback_call_servo_trigger, colorImageCB, netWrenchCB}
* Global Variables: ARUCO_DICT, ARUCO_PARAMS, detector, EBOT_ID
"""

import math
import sys
import time
from functools import partial

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped

# from linkattacher_msgs.srv import AttachLink, DetachLink  # uncomment for sim
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)
from ur_msgs.srv import SetIO

from ebot_docking.srv import PassingService

# Aruco Processing Objects
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
ARUCO_PARAMS = aruco.DetectorParameters()
detector = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# Ebot ID
EBOT_ID = 6  # only for hardware, 12 for sim


class Arm(Node):
    def __init__(self):
        super().__init__("aruco_tf_node")
        # Variables
        self.state = 0

        self.arm_started = False  # toggled after servo trigger
        self.arm_rate = self.create_rate(
            20, self.get_clock()
        )  # sampling rate at which all my controller threads run

        self.cv_image = None
        self.bridge = CvBridge()

        self.attach_done = False  # Flag for attach service
        self.attach_called = False
        self.allow_pick = True  # Flag for passing control from picking timer to passing service request

        self.force = 0.0

        self.box_dict = {}
        self.box_done = []
        self.ebot_pose = []
        self.box_request = False
        self.box_conveyer = None

        # Lookup and Broadcast TF
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.servo_x = 0.0
        self.servo_y = 0.0
        self.servo_z = 0.0
        self.servo_roll = 0.0
        self.servo_pitch = 0.0
        self.servo_yaw = 0.0
        self.servo_looked_up = False

        self.servo_tf_buffer = tf2_ros.Buffer()
        self.servo_tf_listener = tf2_ros.TransformListener(self.servo_tf_buffer, self)

        if EBOT_ID == 6:  # only for hardware
            image_topic = "/camera/camera/color/image_raw"
            vel_topic = "/ServoCmdVel"
        else:
            image_topic = "/camera/color/image_raw"
            vel_topic = "/servo_node/delta_twist_cmds"

        # Subscribers
        self.color_cam_sub = self.create_subscription(
            Image,
            image_topic,
            self.colorImageCB,
            10,
        )

        self.force_sub = self.create_subscription(
            Float64,
            "/net_wrench",
            self.netWrenchCB,
            10,
        )

        # Publishers
        self.image_pub = self.create_publisher(CompressedImage, "/camera/image1048", 10)

        self.servo_pub = self.create_publisher(
            TwistStamped,
            vel_topic,
            10,
        )

        self.image_group = MutuallyExclusiveCallbackGroup()
        self.image_timer = self.create_timer(
            1.0, self.process_image, callback_group=self.image_group
        )

        self.passing_group = MutuallyExclusiveCallbackGroup()
        self.passing_server = self.create_service(
            PassingService,
            "/passing_service",
            self.passingServerCB,
            callback_group=self.passing_group,
        )
        self.pass_done = self.create_publisher(Bool, "/pass_done", 10)

        self.arm_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            0.05, self.arm_handler, callback_group=self.arm_group
        )

        self.call_servo_trigger()

    def passingServerCB(self, request, response):
        """
        Output: srv.PassingService | Tells ebot which conveyer box is supposed to be dropped at
        ---
        Logic: Box request flag tells arm controller to drop the box onto ebot_aruco, and send conveyer number based on box number
        """
        self.box_request = True
        while self.box_conveyer is None:
            pass

        response.conveyer = self.box_conveyer
        return response

    def arm_handler(self):
        if not (self.arm_started and self.box_dict):
            self.get_logger().info("Running arm_handler")
            self.process_image()
            return

        if not self.servo_looked_up:
            self.servo_lookup()
            return

        box_number = list(self.box_dict)[0]
        box_pose = self.box_dict[box_number]
        print(box_number)
        self.box_conveyer = (int(box_number) % 2) + 1

        goal_x, goal_y, goal_z = box_pose
        if EBOT_ID == 6:
            goal_z += 0.1  # only for hardware

        self.saved_z = goal_z + 0.15

        #if box_number != "3":
        goal_x -= 0.06
        goal_y += 0.04

        error_x = 0.0
        error_y = 0.0
        error_z = 0.0
        rot_error = 0.0

        state = self.state

        self.servo_lookup()

        if state == 0:
            rot_error = (
                math.pi - abs(self.servo_roll)
                if self.servo_roll >= 0
                else -math.pi - self.servo_roll
            )
            if abs(rot_error) < 0.05:
                self.state = 1
                rot_error = 0.0
        elif state == 1:
            error_x = goal_x - self.servo_x
            error_y = goal_y - self.servo_y
            error_z = goal_z - self.servo_z

            if abs(error_y) > 0.25:
                error_x = 0.0
                error_z = 0.0

            if all(abs(err) < 0.015 for err in (error_x, error_y, error_z)):
                self.state = 2
                error_x = 0.0
                error_y = 0.0
                error_z = 0.0
        elif state == 2:
            if EBOT_ID == 6:
                error_z = -0.01

                if self.force > 75.0:  # only for hardware
                    error_z = 0.0
                    self.state = 3
            else:
                self.state = 3  # only for software
        elif state == 3:
            if not self.attach_called:
                self.call_attach_box(box_number)
                self.attach_called = True

            if not self.attach_done:
                return

            self.state = 4
        elif state == 4:
            error_z = self.saved_z - self.servo_z
            if abs(error_z) < 0.015:
                self.state = 5
        elif state == 5:
            if not self.box_request:
                return
            if not self.ebot_pose:
                self.process_image()
                return

            error_x = (self.ebot_pose[0] + 0.01) - self.servo_x
            error_y = self.ebot_pose[1] - self.servo_y
            error_z = self.saved_z - self.servo_z

            if all(abs(err) < 0.015 for err in (error_x, error_y, error_z)):
                self.state = 6
                error_x = 0.0
                error_y = 0.0
                error_z = 0.0

            if abs(error_x) > 0.25:
                error_y = 0.0
                if abs(error_z) > 0.015:
                    error_x = 0.0
        elif state == 6:
            error_z = (self.saved_z - 0.1) - self.servo_z
            if abs(error_z) < 0.015:
                self.state = 7
        elif state == 7:
            if self.attach_called:
                self.call_detach_box(box_number)
                self.attach_called = False

            if self.attach_done:
                return

            self.box_request = False

            time.sleep(1.0)
            self.state = 8
        elif state == 8:
            error_z = self.saved_z - self.servo_z
            if abs(error_z) < 0.015:
                self.state = 0

        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        twist_msg.twist.linear.x = error_x * 30.0
        twist_msg.twist.linear.y = error_y * 30.0
        twist_msg.twist.linear.z = error_z * 30.0
        twist_msg.twist.angular.y = rot_error * 45.0

        self.servo_pub.publish(twist_msg)

    # def call_attach_box(self, box_number):
    #     client = self.create_client(AttachLink, "GripperMagnetON")
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn("Waiting for Attach_Link Server...")
    #
    #     request = AttachLink.Request()
    #     request.model1_name = "box" + box_number
    #     request.link1_name = "link"
    #     request.model2_name = "ur5"
    #     request.link2_name = "wrist_3_link"
    #
    #     future = client.call_async(request)
    #     future.add_done_callback(
    #         partial(self.callback_call_attach_box, box_number=box_number)
    #     )

    def call_attach_box(self, box_number):
        """
        Output: future | Does not return the variable, just attaches a callback at service completion
        ---
        Logic: Calls the set_io service. Attaches a callback to address attach completion
        """
        client = self.create_client(SetIO, "/io_and_status_controller/set_io")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("EEF Tool service not available, waiting again...")

        request = SetIO.Request()
        request.fun = 1
        request.pin = 16
        request.state = 1.0

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_attach_box, box_number=box_number)
        )

    def callback_call_attach_box(self, future, box_number):
        """
        Output: null | toggles the attach_done flag
        ---
        Logic: notifies main controller that attach has been done.
        """
        try:
            response = future.result()
            if response.success:
                self.attach_done = True
                self.get_logger().info("Attached Box" + box_number)
        except Exception as e:
            self.get_logger().error(f"Attach Box Client Failed: {e}")

    # def call_detach_box(self, box_number):
    #     client = self.create_client(DetachLink, "GripperMagnetOFF")
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn("Waiting for Detach_Link Server...")
    #
    #     request = DetachLink.Request()
    #     request.model1_name = "box" + box_number
    #     request.link1_name = "link"
    #     request.model2_name = "ur5"
    #     request.link2_name = "wrist_3_link"
    #     future = client.call_async(request)
    #     future.add_done_callback(
    #         partial(self.callback_call_detach_box, box_number=box_number)
    #     )

    def call_detach_box(self, box_number):
        """
        Output: future | Does not return the variable, just attaches a callback at service completion
        ---
        Logic: Calls the set_io service. Attaches a callback to address detach completion
        """
        client = self.create_client(SetIO, "/io_and_status_controller/set_io")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("EEF Tool service not available, waiting...")

        request = SetIO.Request()
        request.fun = 1
        request.pin = 16
        request.state = 0.0

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_detach_box, box_number=box_number)
        )

    def callback_call_detach_box(self, future, box_number):
        """
        Output: null | toggles the attach_done flag, resets ebot_aruco_pose
        ---
        Logic: notifies main controller that detach has been done, removes dropped box from box_pick list, sends pass_done to ebot
        """
        try:
            response = future.result()
            if response.success:
                self.allow_pick = True
                self.attach_done = False
                self.box_done.append(box_number)
                del self.box_dict[box_number]
                self.ebot_pose = []
                self.get_logger().info("Detached Box" + box_number)
                done = Bool()
                done.data = True
                self.pass_done.publish(done)
        except Exception as e:
            self.get_logger().error(f"Detach Box Client Failed: {e}")

    def process_image(self):
        """
        Output: null | Makes changes to class variables, Can be timer called function
        ---
        Logic: Detects aruco in camera frame, broadcasts it's tf w.r.t. base_link, and appends box number and pose to a dictionary. Removes boxes that have already been passed to ebot from the said dictionary, as an extra measure
        """

        if self.cv_image is None:
            return

        result = self.detect_aruco()

        if result is not None:
            translation, angle, ids = result
        else:
            self.get_logger().warn("No Aruco Markers Detected")
            return

        for i in range(0, len(ids)):
            self.broadcast_transform(
                "camera_color_optical_frame",
                # "camera_link",
                "1048_cam_" + str(ids[i]),
                translation[i],
                angle[i],
            )  # Broadcast aruco tf w.r.t camera

            pose, rotation = self.find_transform(
                translation[i],
                angle[i],
            )

            self.broadcast_transform(
                "base_link",
                "1048_base_" + str(ids[i]),
                pose,
                rotation,
            )  # Broadcast box tf w.r.t base

            if ids[i] == EBOT_ID:
                self.ebot_pose = pose
            elif ids[i] < 6:
                self.box_dict[str(ids[i])] = pose

            for done_box in self.box_done:
                self.box_dict.pop(done_box, None)

        return

    def servo_lookup(self):
        """
        Output: null | Makes changes to class servo_* variables
        ---
        Logic: Lookup transform between end effector and base_link.
        """
        try:
            self.servo_transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            )

            self.servo_x, self.servo_y, self.servo_z = (
                self.servo_transform.transform.translation.x,
                self.servo_transform.transform.translation.y,
                self.servo_transform.transform.translation.z,
            )

            quaternion = (
                self.servo_transform.transform.rotation.x,
                self.servo_transform.transform.rotation.y,
                self.servo_transform.transform.rotation.z,
                self.servo_transform.transform.rotation.w,
            )
            self.servo_roll, self.servo_pitch, self.servo_yaw = euler_from_quaternion(
                quaternion
            )

            self.servo_looked_up = True

        except tf2_ros.LookupException:
            self.get_logger().warn(
                "Transform between base_link and wrist_3_link not found."
            )
        except tf2_ros.ConnectivityException:
            self.get_logger().warn(
                "Connectivity issue between base_link and wrist_3_link."
            )
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn(
                "Extrapolation issue between base_link and wrist_3_link."
            )

    def find_transform(self, translation, angles):
        """
        Output: List | Aruco pose w.r.t base link for all detected aruco
        ---
        Logic: Lookup transform between base_link and camera frame to estimate aruco pose of box w.r.t base_link
        """

        try:
            base_to_camera = self.tf_buffer.lookup_transform(
                "base_link", "camera_color_optical_frame", rclpy.time.Time()
            )

            base_translation = [
                base_to_camera.transform.translation.x,
                base_to_camera.transform.translation.y,
                base_to_camera.transform.translation.z,
            ]

            cam_orientation = [
                base_to_camera.transform.rotation.x,
                base_to_camera.transform.rotation.y,
                base_to_camera.transform.rotation.z,
                base_to_camera.transform.rotation.w,
            ]

            base_rotation = tf_transformations.quaternion_multiply(
                cam_orientation, angles
            )

            base_to_camera_matrix = concatenate_matrices(
                translation_matrix(base_translation), quaternion_matrix(cam_orientation)
            )

            camera_to_aruco_matrix = concatenate_matrices(
                translation_matrix(translation), quaternion_matrix(angles)
            )

            base_to_aruco_matrix = concatenate_matrices(
                base_to_camera_matrix, camera_to_aruco_matrix
            )

            deg = tf_transformations.euler_from_quaternion(base_rotation)

            base_rotation = tf_transformations.quaternion_from_euler(
                0.0,
                math.pi,
                (2 * math.pi) + deg[2],
                "sxyz",  # The pitch, is to make z axis point toward inside the box
            )
            aruco_translation_in_base = translation_from_matrix(base_to_aruco_matrix)

            return aruco_translation_in_base.tolist(), base_rotation
        except Exception as e:
            self.get_logger().info(f"Could not calculate transform: {e}")

    def broadcast_transform(self, parent_frame, child_frame, translation, rotation):
        """
        Output: null
        ---
        Logic: Used to broadcast given tf. Allowing aruco estimation in rviz, and lookup by other scripts
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])

        transform.transform.rotation.x = float(rotation[0])
        transform.transform.rotation.y = float(rotation[1])
        transform.transform.rotation.z = float(rotation[2])
        transform.transform.rotation.w = float(rotation[3])

        self.tf_broadcaster.sendTransform(transform)

    def detect_aruco(self):
        """
        Output: 3 Lists | translation, orientation, marker_id of detected aruco
        ---
        Logic: Detect and estimate aruco pose w.r.t. camera frame. Add the same to lists and return for all such detected markers.
        """
        frame = self.cv_image

        translation_aruco_list = []
        angle_aruco_list = []
        marker_ids = []

        camera_matrix = np.array(
            [
                [931.1829833984375, 0.0, 640.0],
                [0.0, 931.1829833984375, 360.0],
                [0.0, 0.0, 1.0],
            ]
        )

        dist_coeffs = np.zeros((5, 1))

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            for corner, marker_id in zip(corners, ids):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner,
                    0.15,
                    camera_matrix,
                    dist_coeffs,  # 0.15 is marker_size
                )

                rvec = np.squeeze(rvec)
                tvec = np.squeeze(tvec)

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                _, buffer = cv2.imencode(".jpg", frame)
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = np.array(buffer).tobytes()
                self.image_pub.publish(msg)
                self.get_logger().info("Published ArUco detected image")

                marker = marker_id[0]
                translation_aruco = tvec
                angle_aruco = [*self.quaternion_from_rotation_vector(rvec)]

                marker_ids.append(marker)
                translation_aruco_list.append(translation_aruco)
                angle_aruco_list.append(angle_aruco)

            return translation_aruco_list, angle_aruco_list, marker_ids

        return None

    def quaternion_from_rotation_vector(self, rvec):
        theta = np.linalg.norm(rvec)
        if theta < 1e-6:
            return np.array([0, 0, 0, 1], dtype=np.float32)
        else:
            axis = rvec / theta
            return np.array(
                [
                    np.sin(theta / 2) * axis[0],
                    np.sin(theta / 2) * axis[1],
                    np.sin(theta / 2) * axis[2],
                    np.cos(theta / 2),
                ],
                dtype=np.float32,
            )

    def call_servo_trigger(self):
        client = self.create_client(Trigger, "/servo_node/start_servo")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Servo Trigger Server...")

        request = Trigger.Request()

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_servo_trigger))

    def callback_call_servo_trigger(self, future):
        try:
            response = future.result()
            if response.success:
                self.arm_started = True
        except Exception as e:
            self.get_logger().error(f"Servo Trigger Call Failed: {e}")

    def colorImageCB(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def netWrenchCB(self, msg):
        self.force = msg.data


def main():
    rclpy.init(args=sys.argv)

    arm_node = Arm()

    executor = MultiThreadedExecutor(3)
    executor.add_node(arm_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
