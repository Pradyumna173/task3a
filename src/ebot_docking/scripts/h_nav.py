#!/usr/bin/env python3

import rclpy, math, time, sys
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
# from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# from my_robot_interfaces.srv import DockSw, PassingService
from ebot_docking.srv import DockSw
from tf_transformations import quaternion_from_euler
from functools import partial
from geometry_msgs.msg import Twist


class EbotNav(Node):
    def __init__(self):

        super().__init__("ebot_navigation_started")
        self.docking_done = False
        self.box_on_bot = "nothing"

        self.pose_dict = {
            "rec": [2.75, -2.8, -1.57],
            "con1": [2.85, 1.83, -1.57],
            "con2": [2.75, -1.28, -1.57],
        }

        self.activity_queue = ["rec", "con1","rec"]
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.handler_group = MutuallyExclusiveCallbackGroup()
        self.client_group = MutuallyExclusiveCallbackGroup()

        self.nav = BasicNavigator()

        initial_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.nav.setInitialPose(initial_pose)

        self.nav.waitUntilNav2Active()

        self.nav_timer = self.create_timer(
            0.5, self.nav_manager, callback_group=self.handler_group
        )

    def create_pose_stamped(self, x, y, yaw=0.0):
        """
        Purpose:
        ---
        Create a `PoseStamped` message representing a goal pose in the "map" frame, with a
        specified position and orientation.

        Input Arguments:
        ---
        `x` : [ float ]
            X-coordinate of the goal position in the "map" frame.

        `y` : [ float ]
            Y-coordinate of the goal position in the "map" frame.

        `yaw` : [ float, optional, default=0.0 ]
            Yaw angle (rotation around the z-axis) of the goal pose in radians.

        Returns:
        ---
        `goal_pose` : [ PoseStamped ]
            A `PoseStamped` message containing the goal pose with the specified position and
            orientation.

        Example call:
        ---
        stamped_pose = create_pose_stamped(1.5, 2.0, yaw=1.57)
        """

        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        # goal_pose.pose.position.z = 0.0
        # goal_pose.pose.orientation.x = 0.0#q_x
        # goal_pose.pose.orientation.y = 0.0#q_y
        goal_pose.pose.orientation.z = 0.0#q_z
        goal_pose.pose.orientation.w = 1.0#q_w
        return goal_pose

    def go_to_goal(self, pose_stamped):
        """
        Purpose:
        ---
        Sends a `PoseStamped` goal to the navigation stack and waits for the ebot to reach
        the target position. Returns whether the navigation task succeeded.

        Input Arguments:
        ---
        `pose_stamped` : [ PoseStamped ]
            The goal pose for the ebot, represented as a `PoseStamped` message.

        Returns:
        ---
        `success` : [ bool ]
            `True` if the ebot successfully reached the goal, `False` otherwise.

        Example call:
        ---
        goal_pose = create_pose_stamped(1.0, 2.0, yaw=1.57)
        success = go_to_goal(goal_pose)
        """

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
        Purpose:
        ---
        Manages navigation tasks by processing an activity queue. Sends the ebot to
        specified goals, performs docking or passing operations based on the activity,
        and ensures completion before moving to the next activity in the queue.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        nav_manager()
        Timer based call made here. Not necessarily timer based function
        """

        if not self.activity_queue:
            return

        goal = self.create_pose_stamped(*self.pose_dict[self.activity_queue[0]])
        if self.go_to_goal(goal):
            if self.activity_queue[0] == "rec":
                self.call_dock(self.activity_queue[0])
                while not self.docking_done:
                    pass
                self.docking_done = False
            else:
                self.call_dock(self.activity_queue[0])
                while not self.docking_done:
                    pass
                self.docking_done = False

            self.activity_queue.pop(0)
            if self.activity_queue:
                forward_msg = Twist()
                forward_msg.linear.x = 0.5
                self.vel_pub.publish(forward_msg)

    def call_dock(self, target):
        client = self.create_client(
            DockSw, "/docking", callback_group=self.client_group
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Docking Server...")

        request = DockSw.Request()
        request.target = target
        request.box_number = self.box_on_bot

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_dock, target=target))

    def callback_call_dock(self, future, target):
        try:
            response = future.result()
            if response:
                self.docking_done = response.success
                self.get_logger().info("Docking Done")
        except Exception as e:
            self.get_logger().error("Docking Call Failed")


def main():
    rclpy.init(args=sys.argv)

    nav_node = EbotNav()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
