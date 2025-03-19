##!/usr/bin/python3
# -*- coding: utf-8 -*-
from cv2 import aruco

# Aruco Processing Objects
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
ARUCO_PARAMS = aruco.DetectorParameters()
detector = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# Ebot ID
EBOT_ID = 12  # 6 for hardware, 12 for sim

BOX_REQUEST = 1  # 0 means do nothing, 1 means
