#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
# Team ID:          1048
# Theme:            Logistic Cobot
# Author List:      Pradyumna Ponkshe, Shantanu Ekad
# Filename:         arm_1048.py
# Functions:        < Comma separated list of functions in this file >
# Global variables: ARUCO_DICT, ARUCO_PARAMS, detector
"""

import rclpy, sys, cv2, math, time
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage, Image
from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_matrix,
    translation_matrix,
    concatenate_matrices,
    translation_from_matrix,
)
from functools import partial
from std_msgs.msg import Float64

# from linkattacher_msgs.srv import AttachLink, DetachLink
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger
from ebot_docking.srv import PassingService

# Aruco Processing Objects
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
DISTANCE_CORRECTION = 0.00  # 0.15


def detect_aruco(image):

    # Empty Arrays made, to fill return at end of function with meaningful values
    translation_aruco_list = []
    angle_aruco_list = []
    marker_ids = []

    # Distortion matrix provided by E-Yantra team for their particular camera/camera-plugin
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
            if marker_id == 6:  # Marker ID Sim is 12. In Hardware it is 6
                MARKER_SIZE = 0.15  # Sim Aruco Size = 0.1275m. Hardware Aruco = 0.15
            else:
                MARKER_SIZE = 0.15

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, MARKER_SIZE, camera_matrix, dist_coeffs
            )

            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec[0], tvec[0], 0.1)

            R, _ = cv2.Rodrigues(
                rvec[0]
            )  # Get rotation matrix of axis orientation of aruco marker
            roll, pitch, yaw = rotation_matrix_to_euler_angles(R)

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

    return translation_aruco_list, angle_aruco_list, marker_ids


def rotation_matrix_to_euler_angles(R):

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

    return (
        np.degrees(roll),
        np.degrees(pitch),
        np.degrees(yaw),
    )  # Converting to degrees for debugging. It is easier to imagine degrees than radian.


class ArucoTF(Node):
    def __init__(self):

        super().__init__("aruco_tf_node")
        # Variables
        self.ur5_engaged = False
        self.current_box = None
        self.servo_started = False
        self.servo_x = None
        self.servo_y = None
        self.servo_z = None

        self.force = 0.0
        self.last_force = 0.0

        self.temp_z = None

        self.start_time = None
        self.box_pressed = False
        self.check_pressed = True

        self.servo_roll = None
        self.servo_pitch = None
        self.servo_yaw = None
        self.place_now = False

        self.image_received = False
        self.cv_image = None
        self.depth_image = None
        self.box_dict = {}  # Will hold box_name: box_pose, gotten by processing image
        self.box_done = []  # List of boxes passed by arm
        self.ebot_pose = []  # Changes when cam sees ebot aruco

        # Callback group declaration
        self.sub_group = ReentrantCallbackGroup()
        self.client_group = MutuallyExclusiveCallbackGroup()
        self.controller_group = MutuallyExclusiveCallbackGroup()
        self.image_group = MutuallyExclusiveCallbackGroup()
        # Server groups
        self.passing_group = MutuallyExclusiveCallbackGroup()
        self.lavda_group = MutuallyExclusiveCallbackGroup()
        self.pick_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.color_cam_sub = self.create_subscription(
            Image,
            # "/camera/color/image_raw",
            "/camera/camera/color/image_raw",
            self.colorimagecb,
            10,
            callback_group=self.sub_group,
        )

        self.force_sub = self.create_subscription(
            Float64,
            "/net_wrench",
            self.force_sub_callback,
            10,
            callback_group=self.sub_group,
        )

        self.depth_cam_sub = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depthimagecb,
            10,
            callback_group=self.sub_group,
        )  # Not in use right now

        # Publishers
        # self.servo_pub = self.create_publisher(
        #     TwistStamped, "/servo_node/delta_twist_cmds", 10
        # )

        self.servo_pub = self.create_publisher(TwistStamped, "/ServoCmdVel", 10)

        # Timers
        self.process_image_timer = self.create_timer(
            1.0, self.process_image, callback_group=self.image_group
        )

        self.servo_lookup_timer = self.create_timer(
            0.05, self.servo_lookup, callback_group=self.controller_group
        )

        #'''
        self.pick_check_timer = self.create_timer(
            1.0, self.pick_box, callback_group=self.pick_group
        )
        #'''

        # Lookup and Broadcast TF
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.servo_tf_buffer = tf2_ros.Buffer()
        self.servo_tf_listener = tf2_ros.TransformListener(self.servo_tf_buffer, self)

        # Cv2 Objects
        self.bridge = CvBridge()

        # Servers
        self.passing_server = self.create_service(
            PassingService,
            "/passing_service",
            self.passing_server_callback,
            callback_group=self.lavda_group,
        )

        self.call_servo_trigger()

    # ---------------------------------------- CLASS END ------------------------------------------
    def passing_server_callback(self, request, response):
        box_number = self.current_box
        self.process_image()
        print("REQUEST RECEIVED")
        while not self.place_now:
            time.sleep(0.5)
            pass

        self.place_now = False
        self.pass_box(box_number)
        time.sleep(1.0)
        response.success = True
        response.box_number = str(box_number)
        return response

    def pick_box(self):

        if self.ur5_engaged:
            self.place_now = True
            return

        if self.image_received:
            self.process_image()
        else:
            return

        if not self.box_dict:
            return

        # print(self.box_dict)

        self.ur5_engaged = True

        box_number = list(self.box_dict)[0]
        box_pose = self.box_dict[box_number]

        print(box_number)

        goal_x, goal_y, goal_z = box_pose
        goal_z += 0.15
        # goal_y += 0.05
        # goal_x += 0.02
        self.temp_z = goal_z

        goal_rot = math.pi

        # Calculate the rotation error based on the sign of the roll
        rot_error = (
            goal_rot - abs(self.servo_roll)
            if self.servo_roll >= 0
            else -goal_rot - self.servo_roll
        )

        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"
        while abs(rot_error) > 0.05:
            rot_error = (
                goal_rot - abs(self.servo_roll)
                if self.servo_roll >= 0
                else -goal_rot - self.servo_roll
            )
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.angular.y = 45.0 * rot_error
            self.servo_pub.publish(twist_msg)
            time.sleep(0.05)

        print("Rotation Corrected")

        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"

        error_x = goal_x - self.servo_x
        error_y = goal_y - self.servo_y
        error_z = goal_z - self.servo_z

        while (abs(error_x) > 0.02) or (abs(error_y) > 0.02) or (abs(error_z) > 0.02):
            error_x = goal_x - self.servo_x
            error_y = goal_y - self.servo_y
            error_z = goal_z - self.servo_z

            twist_msg.header.stamp = self.get_clock().now().to_msg()

            twist_msg.twist.linear.y = 30.0 * error_y
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.z = 0.0

            if abs(error_y) < 0.25:
                twist_msg.twist.linear.x = 30.0 * error_x
                twist_msg.twist.linear.z = 30.0 * error_z

            self.servo_pub.publish(twist_msg)

            time.sleep(0.05)

        print("Goal_reached")
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0

        twist_msg.twist.linear.z = -1.0

        while self.force < 50.0:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.servo_pub.publish(twist_msg)
            time.sleep(0.05)

        print("Gripping Now")

        twist_msg.twist.linear.z = 0.0
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.servo_pub.publish(twist_msg)

        self.picked = False
        self.call_attach_box(box_number)
        while not self.picked:
            pass

        print("Box Gripped")

        self.current_box = box_number
        

    def pass_box(self, box_number):
        while not self.ebot_pose:
            self.process_image()
            pass

        print("Passing Now")

        goal_x, goal_y, goal_z = self.ebot_pose
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"

        error_x = goal_x - self.servo_x
        error_y = goal_y - self.servo_y
        error_z = (self.temp_z + 0.05) - self.servo_z
        # error_z = 0.0

        while (abs(error_x) > 0.02) or (abs(error_y) > 0.02) or (abs(error_z) > 0.02):
            error_x = goal_x - self.servo_x
            error_y = goal_y - self.servo_y
            error_z = (self.temp_z + 0.05) - self.servo_z
            twist_msg.header.stamp = self.get_clock().now().to_msg()

            twist_msg.twist.linear.z = 30.0 * error_z
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            if abs(error_z) < 0.02:
                twist_msg.twist.linear.x = 30.0 * error_x
                if abs(error_x) < 0.3:
                    twist_msg.twist.linear.y = 30.0 * error_y

            self.servo_pub.publish(twist_msg)

            time.sleep(0.03)

        print("Pass Goal Reached")

        self.dropped = False
        self.call_detach_box(self.current_box)
        self.ebot_pose = None
        while not self.dropped:
            pass

        print("Detached Box")

    def servo_lookup(self):
        try:
            self.servo_transform = self.tf_buffer.lookup_transform(
                "base_link", "wrist_3_link", rclpy.time.Time()
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

        if not self.image_received:
            return

        translation, angle, ids = detect_aruco(self.cv_image)

        if not ids:
            return  # Shouldn't be required, but for loop gave me error when translation was empty so...

        for i in range(0, len(ids)):
            translation[i][2] -= DISTANCE_CORRECTION
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
            )  # Lookup base to camera transform and find base to box transform

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

            if ids[i] == 6:  # 12 if simulation, 6 if remote access hardware
                self.ebot_pose = base_to_aruco_pose
            elif ids[i] < 6:
                self.box_dict[str(ids[i])] = base_to_aruco_pose

            for done_box in self.box_done:
                self.box_dict.pop(done_box, None)  # Safely removes the key if it exists

        return

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

    # def call_attach_box(self, box_number):
    #     client = self.create_client(AttachLink, "GripperMagnetON")
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn("Waiting for Attach_Link Server...")

    #     request = AttachLink.Request()
    #     request.model1_name = "box" + box_number
    #     request.link1_name = "link"
    #     request.model2_name = "ur5"
    #     request.link2_name = "wrist_3_link"

    #     future = client.call_async(request)
    #     future.add_done_callback(
    #         partial(self.callback_call_attach_box, box_number=box_number)
    #     )

    def call_attach_box(self, box_number):
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
        try:
            response = future.result()
            if response.success:
                self.picked = True
                self.get_logger().info("Attached Box" + box_number)
        except Exception as e:
            self.get_logger().error("Attach Box Client Failed")

    # def call_detach_box(self, box_number):
    #     client = self.create_client(DetachLink, "GripperMagnetOFF")
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn("Waiting for Detach_Link Server...")

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
        try:
            response = future.result()
            if response.success:
                self.dropped = True
                self.box_done.append(box_number)
                del self.box_dict[box_number]
                self.get_logger().info("Detached Box" + box_number)
                self.ur5_engaged = False
                self.place_now = False
        except Exception as e:
            self.get_logger().error("Detach Box Client Failed")

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
                self.servo_started = True
        except Exception as e:
            self.get_logger().error("Servo Trigger Call Failed")

    # Subscriber Callbacks
    def colorimagecb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.image_received = True
        # print("Image Received")

    def depthimagecb(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )

    def force_sub_callback(self, msg):
        self.force = msg.data


def main():
    rclpy.init(args=sys.argv)

    aruco_tf_node = ArucoTF()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(aruco_tf_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
