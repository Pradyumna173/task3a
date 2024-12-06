#!/home/northee/..platformio/penv/python
# -*- coding: utf-8 -*-

import rclpy, time
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


def detect_aruco(image):
    translation_aruco_list = []
    angle_aruco_list = []
    marker_ids = []

    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

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


    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        for corner, marker_id in zip(corners, ids):
            if marker_id == 12:
                MARKER_SIZE = 0.15
            else:
                MARKER_SIZE = 0.15

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, MARKER_SIZE, camera_matrix, dist_coeffs
            )

            # cv2.aruco.drawDetectedMarkers(image, corners, ids)
            # cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

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
            ]

            marker_ids.append(marker)
            translation_aruco_list.append(translation_aruco)
            angle_aruco_list.append(angle_aruco)
            # self.get_logger().info("Logger chalto")
            print(f"Marker ID: {marker_id[0]}")
            print(f"Translation (x, y, z): {tvec[0][0]}")
            print(f"Rotation (roll, pitch, yaw): {roll:.2f}, {pitch:.2f}, {yaw:.2f}\n")

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
    )


class aruco_tf(Node):

    def __init__(self):

        super().__init__("aruco_tf_publisher")

        self.color_cam_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.colorimagecb,
            10,
        )
        self.depth_cam_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depthimagecb, 10
        )

        image_processing_rate = 0.5
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # self.timer = self.create_timer(image_processing_rate, self.process_image)
        self.timer = None

        self.servo_tf_buffer = tf2_ros.Buffer()
        self.servo_tf_listener = tf2_ros.TransformListener(self.servo_tf_buffer, self)

        self.servo_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )

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
        translation, angle, ids = detect_aruco(self.cv_image)

        for i in range(0, len(ids)):
            self.broadcast_transform(
                "camera_color_optical_frame",
                "1048_cam_" + str(ids[i]),
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
