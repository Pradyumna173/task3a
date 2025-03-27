import time as tm

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import MutuallyExclusiveCallbackGroup, Node
from std_msgs.msg import Float32MultiArray

from ebot_docking.srv import DockSw


class Dock(Node):
    def __init__(self):
        super().__init__("dock_node")

        self.back_left = 0.0
        self.back_right = 0.0

        self.front_left = 0.0
        self.front_right = 0.0

        self.left_front = 0.0
        self.left_back = 0.0

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.dock_group = MutuallyExclusiveCallbackGroup()
        self.dock_server = self.create_service(
            DockSw,
            "/docking",
            self.callback_dock_server,
            callback_group=self.dock_group,
        )

        self.ultra_sub = self.create_subscription(
            Float32MultiArray, "ultrasonic_sensor_std_float", self.ultra_callback, 10
        )

    def callback_dock_server(self, request, response):
        self.get_logger().info("Dock Called")

        while self.back_left == 0.0:
            self.get_logger().warn("Ultrasonic Band")
            tm.sleep(1.0)

        vel_msg = Twist()
        self.get_logger().info("Docking...")

        while (self.back_left > 90.0) and (self.back_right > 90.0):
            vel_msg.linear.x = -0.3
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            tm.sleep(0.03)

        self.get_logger().info("Gotten a bit in...")

        while abs(self.back_left - self.back_right) > 4:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.02 * (self.back_left - self.back_right)
            self.vel_pub.publish(vel_msg)
            tm.sleep(0.03)

        self.get_logger().info("Aligned again....")

        while self.back_left > 21:
            vel_msg.linear.x = -0.004 * self.back_left
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            tm.sleep(0.03)

        self.get_logger().info("Docking Done")

        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)

        response.success = True
        return response

    def ultra_callback(self, msg):
        self.front_right = msg.data[0]
        self.front_left = msg.data[1]

        self.left_front = msg.data[2]
        self.left_back = msg.data[3]

        self.back_right = msg.data[4]
        self.back_left = msg.data[5]


def main(args=None):
    rclpy.init(args=args)

    dock_node = Dock()

    executor = MultiThreadedExecutor(2)
    executor.add_node(dock_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
