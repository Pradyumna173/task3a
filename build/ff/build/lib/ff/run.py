#!/usr/bin/python3
# -*- coding: utf-8 -*-


import sys

import rclpy
from arm import Arm
from nav import Nav
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class Dock(Node):
    def __init__(self):
        super().__init__("dock_node")


def main(args=sys.argv):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    nodes = []

    selected_robots = sys.argv[1:]
    if "arm" in selected_robots or "all" in selected_robots:
        arm_node = Arm()
        nodes.append(arm_node)
        executor.add_node(arm_node)

    if "nav" in selected_robots or "all" in selected_robots:
        nav_node = Nav()
        nodes.append(nav_node)
        executor.add_node(nav_node)

    if "dock" in selected_robots or "all" in selected_robots:
        dock_node = Dock()
        nodes.append(dock_node)
        executor.add_node(dock_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
