##!/usr/bin/python3
# -*- coding: utf-8 -*-

from rclpy.node import Node


class Nav(Node):
    def __init__(self):
        super().__init__("nav_node")
