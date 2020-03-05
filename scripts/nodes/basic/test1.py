#!/usr/bin/env python
import cv2
import argparse
import time

from superros.comm import RosNode
from baxterpkg.robot import BaxterRobot
from baxterpkg.cameras import BaxterCamera
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# ROS node
node = RosNode("test")
node.setHz(node.setupParameter("hz", 30))

# camera
camera = BaxterCamera(node, limb=BaxterCamera.RIGHT)
camera.setting(resolution=(960, 600), exposure=100, gain=40)

# robot
robot = BaxterRobot(node, limb=BaxterRobot.RIGHT)

# move to pose
test_pose = Pose(position=Point(x=0.67,
                                y=-0.64,
                                z=-0.21),
                 orientation=Quaternion(x=1.0,
                                        y=0.0,
                                        z=0.0,
                                        w=0.0))
robot.movep(test_pose)

while node.isActive():

    # get wrist image
    img = camera.getimage()

    if img is not None:
        cv2.imshow("camera", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

    # broadcast target tf
    robot.refresh()

    node.tick()
