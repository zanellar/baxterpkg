#!/usr/bin/env python

import cv2
import argparse

from superros.comm import RosNode
from baxterpkg.robot import BaxterRobot
from baxterpkg.cameras import BaxterCamera
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

''' This script moves both the arm forward with the gripper pointing down and shows the image form hand cameras'''

# ROS node
node = RosNode("test")
node.setHz(node.setupParameter("hz", 30))

# camera
camera_right = BaxterCamera(node, limb=BaxterCamera.RIGHT)
camera_right.setting(resolution=(960, 600), exposure=100, gain=40)
camera_left = BaxterCamera(node, limb=BaxterCamera.LEFT)
camera_left.setting(resolution=(960, 600), exposure=100, gain=40)

# robot
arm_right = BaxterRobot(node, limb=BaxterRobot.RIGHT)
arm_left = BaxterRobot(node, limb=BaxterRobot.LEFT)

# move to pose
test_pose_1 = Pose(position=Point(x=0.60,
                                  y=-0.20,
                                  z=0.20),
                   orientation=Quaternion(x=1.0,
                                          y=0.0,
                                          z=0.0,
                                          w=0.0))

test_pose_2 = Pose(position=Point(x=0.60,
                                  y=0.20,
                                  z=0.20),
                   orientation=Quaternion(x=1.0,
                                          y=0.0,
                                          z=0.0,
                                          w=0.0))
q1r = arm_right.movep(test_pose_1)
q1l = arm_left.movep(test_pose_2)
print(q1r, q1l)

while node.isActive():

    # get wrist image
    imgr = camera_right.getimage()
    imgl = camera_left.getimage()

    if imgr is not None and imgl is not None:
        cv2.imshow("camera right", imgr)
        cv2.imshow("camera left", imgl)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
    node.tick()
