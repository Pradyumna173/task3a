#!/usr/bin/env python3

import rclpy, math, time, sys
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ebot_docking.srv import DockSw
from tf_transformations import quaternion_from_euler
from functools import partial
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class EbotNav(Node):
    def __init__(self):

        super().__init__("ebot_navigation_started")
        self.docking_done = False
        self.box_on_bot = "nothing"

        self.pose_dict = {
            "rec": [2.95, -2.7, 2.84],
            "con1": [2.8, 1.98, 3.14],
            "con2": [2.6, -1.2, 3.14],
        }

        self.activity_queue = ["rec", "con1", "con2"]
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.handler_group = MutuallyExclusiveCallbackGroup()
        self.client_group = MutuallyExclusiveCallbackGroup()
        self.sub_group = MutuallyExclusiveCallbackGroup()

        # self.call_imu_trigger()

        self.orient_sub = self.create_subscription(
            Float32,
            "/orientation",
            self.odom_callback,
            10,
            callback_group=self.sub_group,
        )

        self.nav = BasicNavigator()

        initial_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.nav.setInitialPose(initial_pose)

        self.nav.waitUntilNav2Active()

        self.nav_timer = self.create_timer(
            0.5, self.nav_manager, callback_group=self.handler_group
        )

    def create_pose_stamped(self, x, y, yaw=0.0):

        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)  # ikde badal yaw

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

    def go_to_goal(self, pose_stamped):

        self.nav.goToPose(pose_stamped)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            return True
        else:
            return False

    def nav_manager(self):
        """
        goal_theta = math.pi if self.current_theta > 1.0 else -math.pi

        while abs(goal_theta - self.current_theta) > 0.15:
            goal_theta = math.pi if self.current_theta > 1.0 else -math.pi

            vel_msg = Twist()
            vel_msg.angular.z = (goal_theta - self.current_theta) * 0.8
            self.vel_pub.publish(vel_msg)
            time.sleep(0.05)

        print("Angle corrected")
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
        return
        """

        if not self.activity_queue:
            forward_msg = Twist()
            self.vel_pub.publish(forward_msg)
            time.sleep(0.03)
            return

        goal = self.pose_dict[self.activity_queue[0]]
        stamped_goal = self.create_pose_stamped(*self.pose_dict[self.activity_queue[0]])
        if self.go_to_goal(stamped_goal):
            print("GOAL REACHED")
            '''
            goal_theta = math.pi if self.current_theta > 1.0 else -math.pi

            while abs(goal_theta - self.current_theta) > 0.1:
                goal_theta = math.pi if self.current_theta > 1.0 else -math.pi

                vel_msg = Twist()
                vel_msg.angular.z = (goal_theta - self.current_theta) * 1.0
                self.vel_pub.publish(vel_msg)
                time.sleep(0.05)

            print("Angle corrected")
            '''
            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            time.sleep(1.0)

            self.call_dock()
            while not self.docking_done:
                pass
            self.docking_done = False
            forward_msg = Twist()
            forward_msg.linear.x = 0.5
            self.vel_pub.publish(forward_msg)

            self.activity_queue.pop(0)

    def call_dock(self):
        client = self.create_client(
            DockSw, "/docking", callback_group=self.client_group
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Docking Server...")

        request = DockSw.Request()
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_dock))

    def callback_call_dock(self, future):
        try:
            response = future.result()
            if response:
                self.docking_done = response.success
                self.get_logger().info("Docking Done")
        except Exception as e:
            self.get_logger().error("Docking Call Failed")

    def odom_callback(self, msg):
        self.current_theta = self.normalize_angle(msg.data)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def call_imu_trigger(self):
        client = self.create_client(Trigger, "/reset_imu")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for IMU Trigger Server...")

        request = Trigger.Request()

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_imu_trigger))

    def callback_call_imu_trigger(self, future):
        try:
            response = future.result()
            if response.success:
                self.imu_reset = True
                self.get_logger().info("IMU set to Zero")
        except Exception as e:
            self.get_logger().error("IMU Trigger Call Failed")


def main():
    rclpy.init(args=sys.argv)

    nav_node = EbotNav()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
