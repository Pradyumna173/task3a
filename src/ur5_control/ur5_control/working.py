#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
# Team ID:          1048
# Theme:            Logistic Cobot
# Author List:      Pradyumna Ponkshe, Shantanu Ekad
# Filename:         arm_1048.py
# Functions:        < Comma separated list of functions in this file >
# Global variables: < List of global variables defined in this file, None if no global variables >
"""


import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_matrix,
    translation_matrix,
    concatenate_matrices,
    translation_from_matrix,
)

from linkattacher_msgs.srv import AttachLink, DetachLink
from std_srvs.srv import Trigger


def detect_aruco(image):
    translation_aruco_list = []
    angle_aruco_list = []
    marker_ids = []

    ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    MARKER_SIZE = 0.15

    camera_matrix = np.array(
        [
            [931.1829833984375, 0.0, 640.0],
            [0.0, 931.1829833984375, 360.0],
            [0.0, 0.0, 1.0],
        ]
    )
    dist_coeffs = np.zeros((5, 1))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, ARUCO_DICT, parameters=ARUCO_PARAMS
    )

    if ids is not None:
        for corner, marker_id in zip(corners, ids):
            if marker_id == 12:
                MARKER_SIZE = 0.1275
            else:
                MARKER_SIZE = 0.15

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, MARKER_SIZE, camera_matrix, dist_coeffs
            )

            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

            R, _ = cv2.Rodrigues(rvec[0])
            roll, pitch, yaw = rotation_matrix_to_euler_angles(R)
            if roll < 0:
                roll += 360
            if pitch < 0:
                pitch += 360
            if yaw < 0:
                yaw += 360

            marker = marker_id[0]
            translation_aruco = tvec[0][0]
            angle_aruco = [
                math.radians(roll),
                math.radians(pitch),
                math.radians(yaw),
            ]  # Converting the degrees back to radian because quaternion_from_euler requires so

            marker_ids.append(marker)
            translation_aruco_list.append(translation_aruco)
            angle_aruco_list.append(angle_aruco)

            print(f"Marker ID: {marker_id[0]}")
            print(f"Translation (x, y, z): {tvec[0][0]}")
            print(
                f"Rotation (roll, pitch, yaw): {roll:.2f}, {pitch:.2f}, {yaw:.2f}\n"
            )  # Printing in degrees for debugging

    # cv2.imshow("ArUco Marker Detection", image)
    # cv2.waitKey(1)

    return translation_aruco_list, angle_aruco_list, marker_ids


def rotation_matrix_to_euler_angles(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0

    return (
        np.degrees(roll),
        np.degrees(pitch),
        np.degrees(yaw),
    )  # Converting to degrees for debugging. Easier to imagine degrees than radian.


class aruco_tf(Node):

    def __init__(self):

        super().__init__("aruco_tf_publisher")

        self.color_cam_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.colorimagecb,
            10,
            # Image,
            # "/camera/camera/color/image_raw",
            # self.colorimagecb,
            # 10,
        )
        self.depth_cam_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depthimagecb, 10
        )

        image_processing_rate = 0.5
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(image_processing_rate, self.process_image)

        self.servo_tf_buffer = tf2_ros.Buffer()
        self.servo_tf_listener = tf2_ros.TransformListener(self.servo_tf_buffer, self)

        self.servo_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )

        self.servo_trigger_client = self.create_client(
            Trigger, "/servo_node/start_servo"
        )

        trigger_request = Trigger.Request()
        self.servo_trigger_client.call_async(trigger_request)

        self.servo_timer = self.create_timer(0.05, self.servo_controller)

        self.cv_image = None
        self.depth_image = None
        self.box_dict = {}
        self.box_done = []
        self.pick = True
        self.ebot_pose = [0.54770305, -0.01345971, -0.26956964]

    def depthimagecb(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )

    def colorimagecb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        if not self.timer:
            self.timer = self.create_timer(0.5, self.process_image)

    def find_transform(self, translation, angles):
        try:

            base_to_camera = self.tf_buffer.lookup_transform(
                "base_link", "camera_color_optical_frame", rclpy.time.Time()
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
            print(aruco_translation_in_base)
            return aruco_translation_in_base.tolist()

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

    def process_image(self):
        # if(self.cv_image == None):
        #     return
        # else:
        translation, angle, ids = detect_aruco(self.cv_image)

        for i in range(0, len(ids)):
            self.broadcast_transform(
                "camera_color_optical_frame",
                "cam_" + str(ids[i]),
                translation[i],
                quaternion_from_euler(0.0, 0.0, 0.0, "sxyz"),
            )
            hehe = self.find_transform(
                translation[i],
                quaternion_from_euler(angle[i][0], angle[i][1], angle[i][2], "sxyz"),
            )

            self.broadcast_transform(
                "base_link",
                "1048_base_" + str(ids[i]),
                hehe,
                quaternion_from_euler(
                    0.0, math.pi, (math.pi / 2.0) - angle[i][2], "sxyz"
                ),
            )

            if ids[i] == 12:
                self.ebot_pose = hehe
            else:
                self.box_dict[str(ids[i])] = hehe

            for done_box in self.box_done:
                del self.box_dict[done_box]

        return

    def attach_box(self, box_number):
        attachNode = Node("attach_node")

        attach_client = attachNode.create_client(AttachLink, "GripperMagnetON")
        request = AttachLink.Request()

        while not attach_client.wait_for_service(0.1):
            attachNode.get_logger().warn("Waiting for Gripper Server...")

        request.model1_name = "box" + box_number
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"

        future = attach_client.call_async(request)
        print("ATTACH")
        rclpy.spin_until_future_complete(attachNode, future)

        try:
            response = future.result()
            if response.success:
                self.pick = False
        except Exception as e:
            self.get_logger().error("Service Failed Trying Again...")

    def detach_box(self, box_number):
        detachNode = Node("detach_node")

        detach_client = detachNode.create_client(DetachLink, "GripperMagnetOFF")
        request = DetachLink.Request()

        while not detach_client.wait_for_service(0.1):
            detachNode.get_logger().warn("Waiting for Gripper Server...")

        request.model1_name = "box" + box_number
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"

        future = detach_client.call_async(request)
        print("DETACH")
        rclpy.spin_until_future_complete(detachNode, future)

        try:
            response = future.result()
            if response.success:
                self.pick = True
                self.box_done.append(box_number)
                del self.box_dict[box_number]
        except Exception as e:
            self.get_logger().error("Service Failed Trying Again...")

    def servo_controller(self):
        if self.box_dict:
            first_box = list(self.box_dict)[0]
            box_pose = self.box_dict[first_box]

            goal_x, goal_y, goal_z = box_pose if self.pick else self.ebot_pose

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "wrist_3_link", rclpy.time.Time()
                )

                error_x = goal_x - transform.transform.translation.x
                error_y = goal_y - transform.transform.translation.y
                error_z = (
                    ((goal_z + 0.05) - transform.transform.translation.z)
                    if self.pick
                    else 0.0
                )

                # error_x, error_y, error_z = [error_x, 0.0, 0.0]  # For debugging

                if abs(error_x) < 0.01 and abs(error_y) < 0.01 and abs(error_z) < 0.01:
                    # print("POCHLO")
                    if self.pick:
                        self.attach_box(first_box)
                    else:
                        self.detach_box(first_box)
                    return

                # Calculate orientation errors
                # Desired orientation: x-axis pointing downward, aligned with base_link z-axis
                quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )
                roll, pitch, yaw = euler_from_quaternion(quaternion)
                print(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                )
                # [-0.0085, 0.5809, 0.1334]
                # Orientation error - assuming downward x-axis, adjusting roll and pitch only
                target_rot = math.pi  # x-axis pointing downward
                if roll > 0:
                    rot_error = target_rot - roll
                elif roll < 0:
                    rot_error = (-target_rot) - roll
                else:
                    rot_error = 0.0

                # Apply P control for position
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = "base_link"
                if abs(rot_error) < 0.01:
                    if self.pick:
                        twist_msg.twist.linear.y = 10.0 * error_y
                        twist_msg.twist.linear.z = 10.0 * error_z

                        if (abs(error_y) < 0.01) and (abs(error_z) < 0.01):
                            twist_msg.twist.linear.x = 13.0 * error_x
                    else:
                        twist_msg.twist.linear.x = 13.0 * error_x
                        if abs(error_x) < 0.01:
                            twist_msg.twist.linear.y = 10.0 * error_y
                            twist_msg.twist.linear.z = 10.0 * error_z

                # Apply P control for orientation
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 15.0 * rot_error
                twist_msg.twist.angular.z = 0.0  # Assuming yaw is free to rotate

                self.servo_pub.publish(twist_msg)
                self.get_logger().info(
                    f"Published linear twist: {twist_msg.twist.linear.x:.2f}, {twist_msg.twist.linear.y:.2f}, {twist_msg.twist.linear.z:.2f}"
                )
                self.get_logger().info(
                    f"Published angular twist: {twist_msg.twist.angular.x:.2f}, {twist_msg.twist.angular.y:.2f}, {twist_msg.twist.angular.z:.2f}"
                )

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

        pass


def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node("aruco_tf_process")
    node.get_logger().info("Node created: Aruco tf process")

    aruco_tf_class = aruco_tf()
    rclpy.spin(aruco_tf_class)
    aruco_tf_class.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
