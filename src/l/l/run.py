#!/usr/bin/python3
# -*- coding: utf-8 -*-


import rclpy
from arm import Arm
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class Nav(Node):
    def __init__(self):
        super().__init__("nav_node")


class Dock(Node):
    def __init__(self):
        super().__init__("dock_node")


def main(args=None):
    rclpy.init(args=args)

    arm_node = Arm()
    nav_node = Nav()
    dock_node = Dock()

    executor = SingleThreadedExecutor()
    executor.add_node(arm_node)
    executor.add_node(nav_node)
    executor.add_node(dock_node)

    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
