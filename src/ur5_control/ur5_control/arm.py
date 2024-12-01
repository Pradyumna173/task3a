#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
"""

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
# 			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
# 			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from tf_transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix, euler_from_quaternion
from linkattacher_msgs.srv import AttachLink, DetachLink
from servo_msgs.srv import ServoLink
##################### FUNCTION DEFINITIONS #######################


def calculate_rectangle_area(coordinates):
    """
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    """

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP :
    # 	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library
    #       and use these coordinates to calculate area and width of aruco detected.
    # 	->  Extract values from input set of 4 (x,y) coordinates
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def detect_aruco(image):
    """
    Description:    Function to perform ArUco detection and return each detail of ArUco detected
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image (Image): Input image frame received from respective camera topic

    Returns:
        center_aruco_list (list): Center points of all ArUco markers detected
        distance_from_rgb_list (list): Distance value of each ArUco marker detected from RGB camera
        angle_aruco_list (list): Angle of all pose estimated for ArUco marker
        width_aruco_list (list): Width of all detected ArUco markers
        ids (list): List of all ArUco marker IDs detected in a single frame
    """

    ############ Function VARIABLES ############

    # Threshold value to detect ArUco markers of certain size
    aruco_area_threshold = 500

    # Camera matrix and distortion matrix
    cam_mat = np.array(
        [
            [931.1829833984375, 0.0, 640.0],
            [0.0, 931.1829833984375, 360.0],
            [0.0, 0.0, 1.0],
        ]
    )
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # ArUco marker size in meters
    size_of_aruco_m = 0.15

    # Lists to hold results
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    ############ ADD YOUR CODE HERE ############

    # Convert input BGR image to GRAYSCALE for ArUco detection
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the dictionary for ArUco markers
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    # Define parameters for ArUco detection
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers in the image
    corners, ids, _ = cv2.aruco.detectMarkers(
        gray_image, aruco_dict, parameters=parameters
    )

    # Check if any markers are detected
    if ids is not None:
        for i, corner in enumerate(corners):
            # Calculate the area of the detected marker
            marker_area = cv2.contourArea(corner[0])
            # print(marker_area)

            # Check if the marker area is above the threshold
            if marker_area > aruco_area_threshold:
                # Calculate the center point of the marker
                x_coords = corner[0][:, 0]
                y_coords = corner[0][:, 1]
                center_x = np.mean(x_coords)
                center_y = np.mean(y_coords)
                center_aruco_list.append((center_x, center_y))

                # Estimate pose of the marker
                rvec, tvec, retVal = cv2.aruco.estimatePoseSingleMarkers(
                    corner, size_of_aruco_m, cam_mat, dist_mat
                )

                # Calculate the distance from the camera
                # distance_from_rgb_list.append(np.linalg.norm(tvec[0][0]))
                distance = np.sqrt(np.sum(np.square(tvec[0][0])))
                distance_from_rgb_list.append(distance)

                # Calculate angles (roll, pitch, yaw) from rotation vector
                rvec = rvec[0][0]
                rot_mat, _ = cv2.Rodrigues(rvec)
                angles = cv2.RQDecomp3x3(rot_mat)[0]
                roll, pitch, yaw = angles

                angle_aruco_list.append((roll, pitch, yaw))
                # print(angle_aruco_list)

                # Calculate the width of the marker
                marker_width = np.sqrt(
                    (x_coords[0] - x_coords[1]) ** 2 + (y_coords[0] - y_coords[1]) ** 2
                )
                width_aruco_list.append(marker_width)

                # Draw detected marker and axes on the image
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, size_of_aruco_m)

    # Return the results
    return (
        center_aruco_list,
        distance_from_rgb_list,
        angle_aruco_list,
        width_aruco_list,
        ids,
    )


##################### CLASS DEFINITION #######################


class aruco_tf(Node):
    """
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    """

    def __init__(self):
        """
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        """

        super().__init__("aruco_tf_publisher")  # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.colorimagecb, 10
        )
        self.depth_cam_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depthimagecb, 10
        )

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.5  # rate of time to process image (seconds)
        self.bridge = CvBridge()  # initialise CvBridge object for image conversion
        self.tf_buffer = (
            tf2_ros.buffer.Buffer()
        )  # buffer time used for listening transforms
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(
            self
        )  # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(
            image_processing_rate, self.process_image
        )  # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.servo_tf_buffer = tf2_ros.Buffer()
        self.servo_tf_listener = tf2_ros.TransformListener(self.servo_tf_buffer, self)

        self.servo_timer = self.create_timer(0.1, self.control_loop)

        self.servo_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.cv_image = None  # colour raw image variable (from colorimagecb())
        self.depth_image = None  # depth image variable (from depthimagecb())
        self.box_dict = {}
        self.ebot_drop = [0.54770305, -0.01345971, -0.26956964]
        self.box_done = []
        self.pick = True
        


    def control_loop(self):
        if self.box_dict:
            first_ele = list(self.box_dict)[0]
            print("BOX - ")
            print(self.box_dict[first_ele])
            print("\n EBOT - ")
            print(self.ebot_drop)
            box = self.box_dict[first_ele]

            if self.pick:
                self.target_x = box[0]
                self.target_y = box[1]
                self.target_z = box[2] + 0.025
            else:
                self.target_x = self.ebot_drop[0]
                self.target_y = self.ebot_drop[1]
                self.target_z = self.ebot_drop[2]

            try:
                # Look up transform from base_link to wrist_3_link
                transform = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
                
                # Calculate position errors in x, y, and z
                error_x = self.target_x - transform.transform.translation.x
                error_y = self.target_y - transform.transform.translation.y
                if self.pick:
                    error_z = self.target_z - transform.transform.translation.z
                else:
                    error_z = 0.0
          
                if(abs(error_x) < 0.01 and abs(error_y) < 0.01 and abs(error_z) < 0.01):
                    print("POCHLO")
                    print(self.pick)
                    if self.pick:
                        gripClientNode = Node("attach_node")

                        gripClient = gripClientNode.create_client(AttachLink, "GripperMagnetON")
                        # else:
                        #     client = clientNode.create_client(DetachLink, "GripperMagnetOFF")

                        while not gripClient.wait_for_service(0.1):
                            self.get_logger().warn("Waiting for Gripper Server...")

                        request = AttachLink.Request()  
                        #     request = DetachLink.Request()

                        request.model1_name = "box" + first_ele[1:-1]
                        print(request.model1_name)
                        request.link1_name = 'link'
                        request.model2_name = 'ur5'
                        request.link2_name = 'wrist_3_link'

                        future = gripClient.call_async(request)
                        rclpy.spin_until_future_complete(gripClientNode, future)
                        # print("zaala")

                        try:
                            response = future.result()
                            if response.success:
                                self.pick = False
                        except Exception as e:
                            self.get_logger().error("Service Failed Trying Again...")    
                    else:
                        gripClientNode = Node("attach_node")

                        gripClient = gripClientNode.create_client(DetachLink, "GripperMagnetOFF")
                       

                        while not gripClient.wait_for_service(0.1):
                            self.get_logger().warn("Waiting for Gripper Server...")

                        request = DetachLink.Request()

                        request.model1_name = "box" + first_ele[1:-1]
                        request.link1_name = 'link'
                        request.model2_name = 'ur5'
                        request.link2_name = 'wrist_3_link'

                        future = gripClient.call_async(request)
                        rclpy.spin_until_future_complete(gripClientNode, future)
                        # print("zaala")

                        try:
                            response = future.result()
                            if response.success:
                                self.pick = True
                        except Exception as e:
                            self.get_logger().error("Service Failed Trying Again...")

                        remNode = Node("servo_box_remove")
                        rem_box = remNode.create_client(ServoLink, '/SERVOLINK')
                        while not rem_box.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('Servo service not available, waiting again...')

                        req = ServoLink.Request()
                        req.box_name = "box" + first_ele[1:-1]
                        req.box_link = 'link'
                        future = rem_box.call_async(req)
                        rclpy.spin_until_future_complete(remNode, future)

                        try:
                            response = future.result()
                            if response.success:
                                del self.box_dict[first_ele]
                        except Exception as e:
                            self.get_logger().error("Service Failed Trying Again...")
                
                # Calculate orientation errors
                # Desired orientation: x-axis pointing downward, aligned with base_link z-axis
                quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                roll, pitch, yaw = euler_from_quaternion(quaternion)
                print(transform.transform.translation.x, 
                    transform.transform.translation.y, 
                    transform.transform.translation.z)
                #[-0.0085, 0.5809, 0.1334]
                # Orientation error - assuming downward x-axis, adjusting roll and pitch only
                target_rot = math.pi  # x-axis pointing downward
                if(roll > 0):
                    rot_error = target_rot - roll
                elif(roll < 0):
                    rot_error = (-target_rot) - roll
                else:
                    rot_error = 0.0

                # Apply P control for position
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = 'base_link'
                if(abs(rot_error) < 0.01):
                    twist_msg.twist.linear.x = 13.0 * error_x
                    # twist_msg.twist.linear.x = 1.0
                    twist_msg.twist.linear.y = 13.0 * error_y
                    twist_msg.twist.linear.z = 13.0 * error_z
                
                # Apply P control for orientation
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 5.0 * rot_error
                twist_msg.twist.angular.z = 0.0  # Assuming yaw is free to rotate

                # Publish the twist command
                # self.servo_pub.publish(twist_msg)
                
                self.get_logger().info(f'Published linear twist: {twist_msg.twist.linear.x:.2f}, {twist_msg.twist.linear.y:.2f}, {twist_msg.twist.linear.z:.2f}')
                self.get_logger().info(f'Published angular twist: {twist_msg.twist.angular.x:.2f}, {twist_msg.twist.angular.y:.2f}, {twist_msg.twist.angular.z:.2f}')
            
            except tf2_ros.LookupException:
                self.get_logger().warn('Transform between base_link and wrist_3_link not found.')
            except tf2_ros.ConnectivityException:
                self.get_logger().warn('Connectivity issue between base_link and wrist_3_link.')
            except tf2_ros.ExtrapolationException:
                self.get_logger().warn('Extrapolation issue between base_link and wrist_3_link.')

    def depthimagecb(self, data):
        """
        Description:    Callback function for aligned depth camera topic.
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        """

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        # 	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )
        # cv2.imshow("Image Window", self.depth_image)
        # cv2.waitKey(1)  # Display the image for 1 ms

    def colorimagecb(self, data):
        """
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        """

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        # 	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # cv2.imshow("Image Window", self.cv_image)
        # cv2.waitKey(1)  # Display the image for 1 ms
        
    def find_aruco_transform(self, marker_id, correction):
        try:
            # Step 1: Get transform from base_link to camera_link
            base_to_camera = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())

            # Step 2: Get transform from camera_link to aruco_box (published by RealSense camera)
            # camera_to_aruco = self.tf_buffer.lookup_transform('camera_link', 'aruco_box', rclpy.time.Time())

            # Step 3: Extract translation and rotation from base to camera
            base_translation = [
                base_to_camera.transform.translation.x,
                base_to_camera.transform.translation.y,
                base_to_camera.transform.translation.z
            ]
            base_rotation = [
                base_to_camera.transform.rotation.x,
                base_to_camera.transform.rotation.y,
                base_to_camera.transform.rotation.z,
                base_to_camera.transform.rotation.w
            ]

            # Step 4: Extract translation and rotation from camera to aruco
            aruco_translation = [
                self.x,
                self.y,
                self.z
            ]
            aruco_rotation = [
                self.q[0],
                self.q[1],
                self.q[2],
                self.q[3]
            ]

            # Step 5: Create transformation matrices
            base_to_camera_matrix = concatenate_matrices(
                translation_matrix(base_translation),
                quaternion_matrix(base_rotation)
            )
            
            camera_to_aruco_matrix = concatenate_matrices(
                translation_matrix(aruco_translation),
                quaternion_matrix(aruco_rotation)
            )

            # Step 6: Calculate full transform from base_link to aruco_box
            base_to_aruco_matrix = concatenate_matrices(base_to_camera_matrix, camera_to_aruco_matrix)

            # Step 7: Extract the final translation and rotation for Aruco box in base_link frame
            aruco_translation_in_base = translation_from_matrix(base_to_aruco_matrix)
            aruco_rotation_in_base = quaternion_from_matrix(base_to_aruco_matrix)


            marker_to_base_transform = TransformStamped()
            marker_to_base_transform.header.stamp = self.get_clock().now().to_msg()
            marker_to_base_transform.header.frame_id = "base_link"
            marker_to_base_transform.child_frame_id = "obj_" + (str(marker_id))[1:-1]
            marker_to_base_transform.transform.translation.x = (
                aruco_translation_in_base[0]
            )
            marker_to_base_transform.transform.translation.y = (
                aruco_translation_in_base[1]
            )
            marker_to_base_transform.transform.translation.z = (
                aruco_translation_in_base[2] + correction
            )

            if(marker_id == 12):
                marker_to_base_transform.transform.rotation.x = self.eq1[0]
                marker_to_base_transform.transform.rotation.y = self.eq1[1]
                marker_to_base_transform.transform.rotation.z = self.eq1[2] 
                marker_to_base_transform.transform.rotation.w = self.eq1[3] 
            elif(aruco_translation_in_base[1] > 0.0):
                marker_to_base_transform.transform.rotation.x = self.uq1[0]
                marker_to_base_transform.transform.rotation.y = self.uq1[1]
                marker_to_base_transform.transform.rotation.z = self.uq1[2] 
                marker_to_base_transform.transform.rotation.w = self.uq1[3] 
            else:
                marker_to_base_transform.transform.rotation.x = self.q1[0]
                marker_to_base_transform.transform.rotation.y = self.q1[1]
                marker_to_base_transform.transform.rotation.z = self.q1[2] 
                marker_to_base_transform.transform.rotation.w = self.q1[3] 
            
            # marker_to_base_transform.transform.rotation = base_to_camera_transform.transform.rotation

            # Broadcast the transform from base_link to obj_marker
            self.tf_broadcaster.sendTransform(marker_to_base_transform)
            if(marker_id == 12):
                self.ebot_drop = aruco_translation_in_base
            else:
                self.box_dict[str(marker_id)] = [
                    aruco_translation_in_base[0],
                    aruco_translation_in_base[1],
                    aruco_translation_in_base[2],
                ]
            # print(self.box_dict)
                

            # Log the results
            self.get_logger().info(f"Aruco box in base_link frame: x={aruco_translation_in_base[0]}, "
                                   f"y={aruco_translation_in_base[1]}, z={aruco_translation_in_base[2]}")
            # self.get_logger().info(f"Rotation (quaternion): x={aruco_rotation_in_base[0]}, "
            #                        f"y={aruco_rotation_in_base[1]}, z={aruco_rotation_in_base[2]}, "
            #                        f"w={aruco_rotation_in_base[3]}")
            
        except Exception as e:
            self.get_logger().info(f'Could not calculate transform: {e}')


    
    def get_coordinates(self, target_frame):
        try:
            source_frame = "base_link"
            target_frame = "cam_" + str(target_frame)

            transform = self.tf_buffer.lookup_transform(
                source_frame, target_frame, rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            self.get_logger().info(
                f"Aruco box coordinates relative to base_link: x={x}, y={y}, z={z}"
            )
            return x, y, z

        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")
            return None, None, None

    def process_image(self):
        """
        Description:    Timer function used to detect ArUco markers and publish TF on estimated poses.
        Args:
        Returns:
        """

        ############ Function VARIABLES ############

        # Camera parameters
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        if self.cv_image is not None:
            (
                center_aruco_list,
                distance_from_rgb_list,
                angle_aruco_list,
                width_aruco_list,
                ids,
            ) = detect_aruco(self.cv_image)
            # print(ids)
            # print("\n\n")
            # print(distance_from_rgb_list)
            if center_aruco_list:
                for idx, marker_id in enumerate(ids):
                    # Extract data for each marker
                    cX, cY = center_aruco_list[idx]
                    distance_from_rgb = distance_from_rgb_list[idx]
                    angle_aruco = angle_aruco_list[idx]

                    # Ensure angle_aruco is a single numeric value in radians
                    if isinstance(angle_aruco, (list, tuple)):
                        angle_aruco = angle_aruco[
                            2
                        ]  # Assuming the first element is the angle
                    elif not isinstance(angle_aruco, (float, int)):
                        self.get_logger().error(
                            f"Unexpected type for angle_aruco: {type(angle_aruco)}"
                        )
                        continue

                    # Apply the angle correction formula (assuming angle_aruco is in radians)
                    angle_aruco_corrected = (0.788 * angle_aruco) - (
                        (angle_aruco**2) / 3160
                    )
                    # angle_aruco_corrected = math.radians(angle_aruco)
                    #
                    roll, pitch, yaw = 0, 0, angle_aruco_corrected

                    self.q = quaternion_from_euler(roll, pitch, yaw)
                    self.q1 = quaternion_from_euler(-math.pi, pitch, yaw - (math.pi / 2.0))
                    self.uq1 = quaternion_from_euler(-math.pi, pitch, yaw - (math.pi / 2.0))
                    self.eq1 = quaternion_from_euler(-math.pi, pitch, -math.pi / 2.0)

                    # Calculate (x, y, z) based on focal length, center value, and image size
                    self.y = (cX - centerCamX) * -distance_from_rgb / focalX
                    self.z = (cY - centerCamY) * -distance_from_rgb / focalY

                    self.x = distance_from_rgb - 0.12

                    # Draw circle on the detected ArUco marker center
                    cv2.circle(self.cv_image, (int(cX), int(cY)), 10, (0, 255, 0), -1)

                    # Create a TransformStamped message
                    transform = TransformStamped()
                    transform.header.stamp = self.get_clock().now().to_msg()
                    transform.header.frame_id = "camera_link"
                    transform.child_frame_id = "cam_" + (str(marker_id))[1:-1]
                    if(marker_id == 12):
                        self.x = self.x - 0.3
                        self.z = self.z + 0.1
                    transform.transform.translation.x = self.x
                    transform.transform.translation.y = self.y
                    transform.transform.translation.z = self.z

                    # Compute quaternion from the corrected angle (yaw only)
                    transform.transform.rotation.x = self.q[0]
                    transform.transform.rotation.y = self.q[1]
                    transform.transform.rotation.z = self.q[2]
                    transform.transform.rotation.w = self.q[3]

                    # Broadcast the transform from camera_link to cam_marker
                    self.tf_broadcaster.sendTransform(transform)

                    # Lookup transform from base_link to camera_link
                    # self.box_pos(marker_id)
                    if(marker_id == 12):
                        self.find_aruco_transform(marker_id, 0.0)
                    else:
                        self.find_aruco_transform(marker_id, 0.0)

            # Display the image with the detected markers
            # cv2.imshow("Detected ArUco Markers", self.cv_image)
            cv2.waitKey(1)


##################### FUNCTION DEFINITION #######################


def main():
    """
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    """

    rclpy.init(args=sys.argv)  # initialisation

    node = rclpy.create_node("aruco_tf_process")  # creating ROS node

    node.get_logger().info("Node created: Aruco tf process")  # logging information

    aruco_tf_class = aruco_tf()  # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)  # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()  # destroy node after spin ends

    rclpy.shutdown()  # shutdown process


if __name__ == "__main__":
    """
    Description:    If the python interpreter is running that module (the source file) as the main program,
                    it sets the special __name__ variable to have a value “__main__”.
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    """

    main()
