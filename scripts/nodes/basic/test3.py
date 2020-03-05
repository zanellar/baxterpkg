#!/usr/bin/env python

import cv2
import argparse
import PyKDL

from superros.comm import RosNode
from baxterpkg.robot import BaxterRobot
from baxterpkg.cameras import BaxterCamera
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# ROS node
node = RosNode("test")
node.setHz(node.setupParameter("hz", 10))

# camera
camera_right = BaxterCamera(node, limb=BaxterCamera.RIGHT)
camera_right.setting(resolution=(960, 600), exposure=100, gain=40)

# robot class
arm_right = BaxterRobot(node, limb=BaxterRobot.RIGHT)

Ftr_eef = PyKDL.Frame()
Ftr_eef.p = PyKDL.Vector(0.01, 0.0, 0)

key = raw_input(".............. reset? [y]/n: ")
if str(key) not in ["n", "N", "no", "NO"]:
    arm_right.reset()

while node.isActive():

    # get current pose
    F_eef2base = arm_right.gettf()

    # move right arm up
    Ftr_base = F_eef2base * Ftr_eef
    arm_right.movep(Ftr_base)

    # get wrist image
    imgr = camera_right.getimage()

    if imgr is not None:
        cv2.imshow("camera right", imgr)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
    node.tick()
