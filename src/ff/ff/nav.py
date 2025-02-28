#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy, time, sys
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
from ebot_docking.srv import DockSw, PassingService
from functools import partial


class Nav(Node):
    def __init__(self):
        super().__init__("nav_started")

        self.pose_dict = {
            "rec": [2.95, -2.72, 2.95],
            "con1": [2.8, 1.96, 3.14],
            "con2": [2.6, -1.2, 3.14],
        }  # only for hardware

        # self.pose_dict = {
        #     "rec": [0.4, -2.4, 3.14],
        #     "con1": [-4.0, 2.89, -1.57],
        #     "con2": [2.32, 2.55, -1.57],
        # } # only for sim

        self.activity_queue = ["rec"]

        self.dock_done = False
        self.pass_done = False

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.nav_group = MutuallyExclusiveCallbackGroup()

        self.nav = BasicNavigator()

        self.nav.waitUntilNav2Active()

        self.pass_sub = self.create_subscription(
            Bool, "/pass_done", self.pass_sub_CB, 10
        )

        self.nav_timer = self.create_timer(
            1.0, self.nav_manager, callback_group=self.nav_group
        )

    def nav_manager(self):
        """
        Output: null | Timer based function call
        ---
        Logic: Latches a thread till pre-dock pose is reached by Simple_Commander, then ensure docking is done, then send ebot forward and give next pose.
        """
        if not self.activity_queue:
            return

        stamped_goal = self.create_pose_stamped(*self.pose_dict[self.activity_queue[0]])

        if self.go_to_goal(stamped_goal):
            print("Pre Dock Reached")

            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            time.sleep(1.0)

            self.call_dock()
            while not self.dock_done:
                pass
            self.dock_done = False

            if self.activity_queue[0] == "rec":
                self.call_passing_service()
                while not self.pass_done:
                    pass
                self.pass_done = False
                time.sleep(3.0)

            forward_msg = Twist()
            forward_msg.linear.x = 0.6
            self.vel_pub.publish(forward_msg)

            self.activity_queue.pop(0)

    def call_passing_service(self):
        client = self.create_client(PassingService, "/passing_service")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Passing Service...")

        request = PassingService.Request()

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_passing_service))

    def callback_call_passing_service(self, future):
        try:
            response = future.result()
            next_goal = response.conveyer
            self.activity_queue.append("con" + str(next_goal))
            self.activity_queue.append("rec")
        except Exception as e:
            self.get_logger().error(f"Passing Service Call Failed: {e}")

    def call_dock(self):
        client = self.create_client(DockSw, "/docking")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Docking Server...")

        request = DockSw.Request()
        request.target = self.activity_queue[0]

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_dock))

    def callback_call_dock(self, future):
        response = future.result()
        if response.success:
            self.dock_done = True
            self.get_logger().info("Docking Done")

    def go_to_goal(self, pose_stamped):
        """
        Output: boolean | Whether nav2 goal was reached or not
        ---
        Logic: Just give nav2 goal to Simple_Commander API
        """

        self.nav.goToPose(pose_stamped)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            return True
        else:
            return False

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

    def pass_sub_CB(self, msg):
        self.pass_done = msg.data


def main():
    rclpy.init(args=sys.argv)

    nav_node = Nav()

    executor = MultiThreadedExecutor()
    executor.add_node(nav_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
