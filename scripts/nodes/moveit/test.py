#!/usr/bin/env python
import cv2
import argparse
import time

from superros.comm import RosNode
from baxterpkg.robot import BaxterMoveit
from baxterpkg.cameras import BaxterCamera
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# ROS node
node = RosNode("test")
node.setHz(node.setupParameter("hz", 30))

# robot
robot = BaxterMoveit(node, limb=BaxterMoveit.RIGHT)

# move to pose
test_pose_1 = Pose(position=Point(x=0.72,
                                  y=-0.3,
                                  z=-0.08),
                   orientation=Quaternion(x=1.0,
                                          y=0.0,
                                          z=0.0,
                                          w=0.0))

test_pose_2 = Pose(position=Point(x=0.72,
                                  y=-0.5,
                                  z=-0.08),
                   orientation=Quaternion(x=1.0,
                                          y=0.0,
                                          z=0.0,
                                          w=0.0))


robot.scene.remove_world_object("table")
# robot.scene.remove_world_object("box")
time.sleep(1)
robot.obstacle("table", position=[0.96, -0.355, -0.563], size=[0.75, 1.35, 0.705])
# robot.obstacle("box", position=[0.8325, -0.2725, -0.1675], size=[0.395, 0.27, 0.15])
time.sleep(1)


robot.showplan(test_pose_1)
robot.movep(test_pose_1, raw=False)

while node.isActive():

    # wave
    z = float(raw_input("z="))
    test_tf = Pose(position=Point(x=0.687,
                                  y=-0.5,
                                  z=-z),
                   orientation=Quaternion(x=1.0,
                                          y=0.0,
                                          z=0.0,
                                          w=0.0))
    robot.showplan(test_tf)
    robot.movep(test_tf, raw=False)

    # robot.showplan(test_pose_1)
    # robot.movep(test_pose_1, raw=False)
    # robot.showplan(test_pose_2)
    # robot.movep(test_pose_2, raw=False)

    # broadcast target tf
    robot.refresh()

    node.tick()
