#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
from functools import partial

import cv2
import globals
import numpy as np
import tf2_ros
from cv2 import Rodrigues, aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


class Arm(Node):
    def __init__(self):
        super().__init__("arm_node")

        if globals.EBOT_ID == 6:  # Hardware
            servo_topic = "/ServoCmdVel"
            image_topic = "/camera/camera/color/image_raw"
        else:  # Sim
            servo_topic = "/servo_node/delta_twist_cmds"
            image_topic = "/camera/color/image_raw"

        self.servo_trigger = False
        self.arm_toggle = False
        self.state = 0

        self.box_dict = {}
        self.box_done = []
        self.ebot_pose = []

        # Lookup and Broadcast TF
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publisher Objects

        self.image_pub = self.create_publisher(CompressedImage, "/camera/image1048", 10)

        self.servo_pub = self.create_publisher(
            TwistStamped,
            servo_topic,
            10,
        )
        # Subscriber Objects

        self.cv_image = None

        self.image_sub = self.create_subscription(
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

        # Timer
        self.handler_timer = self.create_timer(2.0, self.timer_handler)
        self.arm_timer = None

        self.bridge = CvBridge()

        self.call_servo_trigger()
        # --------------------------- End of Constructor --------------------------------------

    def timer_handler(self):
        if (globals.BOX_REQUEST == 1) and (self.ebot_pose) and (self.state == 5):
            self.arm_timer = self.create_timer(0.05, self.arm_handler)

    def arm_handler(self):
        self.servo_lookup()

        if not self.box_dict:
            return

        box_number = list(self.box_dict)[0]
        box_pose = self.box_dict[box_number]
        print(box_number)
        self.box_conveyer = (int(box_number) % 2) + 1

        goal_x, goal_y, goal_z = box_pose
        self.saved_z = goal_z + 0.15

        if globals.EBOT_ID == 6:
            goal_x -= 0.06
            goal_y += 0.04
            goal_z += 0.1  # only for hardware
        else:
            goal_z += 0.05

        error_x = 0.0
        error_y = 0.0
        error_z = 0.0
        rot_error = 0.0

        state = self.state

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
            if globals.EBOT_ID == 6:
                error_z = -0.01
                if self.force > 75.0:  # only for hardware
                    error_z = 0.0
                    self.state = 3
            else:
                self.state = 3  # only for software
        elif state == 3:
            self.call_attach_box(box_number)
            if self.arm_timer:
                self.arm_timer.destroy()
        elif state == 4:
            error_z = self.saved_z - self.servo_z
            if abs(error_z) < 0.015:
                self.state = 5
        elif state == 5:
            if not self.ebot_pose:
                if self.arm_timer:
                    self.arm_timer.destroy()
                return

            error_x = (self.ebot_pose[0] + 0.015) - self.servo_x
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
            self.call_detach_box(box_number)
            if self.arm_timer:
                self.arm_timer.destroy()
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

    def servo_lookup(self):
        try:
            self.servo_transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", time.Time()
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

    def process_image(self):
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
                quaternion_from_euler(0.0, 0.0, 0.0, "sxyz"),
            )  # Broadcast aruco tf w.r.t camera

            base_to_aruco_pose = self.find_transform(
                translation[i],
                quaternion_from_euler(angle[i][0], angle[i][1], angle[i][2], "sxyz"),
            )

            self.broadcast_transform(
                "base_link",
                "1048_base_" + str(ids[i]),
                base_to_aruco_pose,
                quaternion_from_euler(
                    0.0,
                    math.pi,
                    (math.pi / 2.0) - angle[i][2],
                    "sxyz",  # The pitch, is to make z axis point toward inside the box
                ),
            )  # Broadcast box tf w.r.t base

            if ids[i] == globals.EBOT_ID:
                self.ebot_pose = base_to_aruco_pose
            elif ids[i] < 6:
                self.box_dict[str(ids[i])] = base_to_aruco_pose

            for done_box in self.box_done:
                self.box_dict.pop(done_box, None)

        return

    def find_transform(self, translation, angles):
        try:
            base_to_camera = self.tf_buffer.lookup_transform(
                "base_link", "camera_color_optical_frame", time.Time()
            )

            base_translation = [
                base_to_camera.transform.translation.x,
                base_to_camera.transform.translation.y,
                base_to_camera.transform.translation.z,
            ]
            base_rotation = [
                base_to_camera.transform.rotation.x,
                base_to_camera.transform.rotation.y,
                base_to_camera.transform.rotation.z,
                base_to_camera.transform.rotation.w,
            ]

            aruco_translation = translation
            aruco_rotation = angles

            base_to_camera_matrix = concatenate_matrices(
                translation_matrix(base_translation), quaternion_matrix(base_rotation)
            )

            camera_to_aruco_matrix = concatenate_matrices(
                translation_matrix(aruco_translation), quaternion_matrix(aruco_rotation)
            )

            base_to_aruco_matrix = concatenate_matrices(
                base_to_camera_matrix, camera_to_aruco_matrix
            )

            aruco_translation_in_base = translation_from_matrix(base_to_aruco_matrix)
            return (
                aruco_translation_in_base.tolist()
            )  # convert numpy array to python list for easier parsing

        except Exception as e:
            self.get_logger().info(f"Could not calculate transform: {e}")

    def broadcast_transform(self, parent_frame, child_frame, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(transform)

    def detect_aruco(self):
        inFrame = self.cv_image
        frame = cv2.GaussianBlur(inFrame, (5, 5), 0)

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

        corners, ids, _ = globals.detector.detectMarkers(frame)

        if ids is not None:
            for corner, marker_id in zip(corners, ids):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner,
                    0.15,
                    camera_matrix,
                    dist_coeffs,  # 0.15 is marker_size
                )

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                _, buffer = cv2.imencode(".jpg", frame)
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = np.array(buffer).tobytes()
                self.image_pub.publish(msg)
                self.get_logger().info("Published ArUco detected image")

                R, _ = Rodrigues(rvec[0])

                marker = marker_id[0]
                translation_aruco = tvec[0][0]
                angle_aruco = [*self.rotation_matrix_to_euler_angles(R)]

                marker_ids.append(marker)
                translation_aruco_list.append(translation_aruco)
                angle_aruco_list.append(angle_aruco)

            return translation_aruco_list, angle_aruco_list, marker_ids

        return None

    def rotation_matrix_to_euler_angles(self, R):
        """
        Output: List | roll, pitch, yaw for given rotation matrix
        ---
        Logic: Converts rotation matrix to legible roll, pitch, yaw
        """
        sy = math.sqrt(
            R[0, 0] ** 2 + R[1, 0] ** 2
        )  # Calculate the magnitude of the first two elements in the first column of R

        singular = sy < 1e-6  # Check for a singularity (gimbal lock)

        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:  # Handle singularity where pitch is limited to +-90 degrees
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0

        return (roll, pitch, yaw)

    # Clients

    def call_attach_box(self, box_number):
        client = self.create_client(AttachLink, "GripperMagnetON")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Attach_Link Server...")

        request = AttachLink.Request()
        request.model1_name = "box" + box_number
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_attach_box, box_number=box_number)
        )

    # def call_attach_box(self, box_number):
    #     """
    #     Output: future | Does not return the variable, just attaches a callback at service completion
    #     ---
    #     Logic: Calls the set_io service. Attaches a callback to address attach completion
    #     """
    #     client = self.create_client(SetIO, "/io_and_status_controller/set_io")
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info("EEF Tool service not available, waiting again...")
    #
    #     request = SetIO.Request()
    #     request.fun = 1
    #     request.pin = 16
    #     request.state = 1.0
    #
    #     future = client.call_async(request)
    #     future.add_done_callback(
    #         partial(self.callback_call_attach_box, box_number=box_number)
    #     )

    def callback_call_attach_box(self, future, box_number):
        """
        Output: null | toggles the attach_done flag
        ---
        Logic: notifies main controller that attach has been done.
        """
        try:
            response = future.result()
            if response.success:
                self.state += 1
                self.arm_timer = self.create_timer(0.05, self.arm_handler)
                self.get_logger().info("Attached Box" + box_number)
        except Exception as e:
            self.get_logger().error(f"Attach Box Client Failed: {e}")

    def call_detach_box(self, box_number):
        client = self.create_client(DetachLink, "GripperMagnetOFF")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Detach_Link Server...")

        request = DetachLink.Request()
        request.model1_name = "box" + box_number
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_detach_box, box_number=box_number)
        )

    # def call_detach_box(self, box_number):
    #     """
    #     Output: future | Does not return the variable, just attaches a callback at service completion
    #     ---
    #     Logic: Calls the set_io service. Attaches a callback to address detach completion
    #     """
    #     client = self.create_client(SetIO, "/io_and_status_controller/set_io")
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info("EEF Tool service not available, waiting...")
    #
    #     request = SetIO.Request()
    #     request.fun = 1
    #     request.pin = 16
    #     request.state = 0.0
    #
    #     future = client.call_async(request)
    #     future.add_done_callback(
    #         partial(self.callback_call_detach_box, box_number=box_number)
    #     )

    def callback_call_detach_box(self, future, box_number):
        """
        Output: null | toggles the attach_done flag, resets ebot_aruco_pose
        ---
        Logic: notifies main controller that detach has been done, removes dropped box from box_pick list, sends pass_done to ebot
        """
        try:
            response = future.result()
            if response.success:
                self.state += 1
                self.arm_timer = self.create_timer(0.05, self.arm_handler)
                globals.BOX_REQUEST = 3

                self.box_done.append(box_number)
                del self.box_dict[box_number]
                self.ebot_pose = []
                self.get_logger().info("Detached Box" + box_number)

        except Exception as e:
            self.get_logger().error(f"Detach Box Client Failed: {e}")

    def call_servo_trigger(self):
        client = self.create_client(Trigger, "/servo_node/start_servo")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Servo Trigger Server...")

        request = Trigger.Request()

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_servo_trigger))

    def callback_call_servo_trigger(self, future):
        try:
            self.get_logger().info("Servo Triggered")
            self.arm_timer = self.create_timer(0.05, self.arm_handler)
        except Exception as e:
            self.get_logger().error(f"Servo Trigger Call Failed: {e}")

    # Subscribers
    def netWrenchCB(self, msg):
        self.force = msg.data

    def colorImageCB(self, msg):
        if self.cv_image is None:
            self.image_timer = self.create_timer(1.0, self.process_image)
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
