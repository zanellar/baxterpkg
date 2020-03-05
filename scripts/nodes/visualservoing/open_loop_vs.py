#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import cv2
import argparse
import PyKDL
import numpy as np
import smach
import time
import math

from superros.comm import RosNode
from superros.transformations import KLDFrameToNumpyRotation, KLDFrameToNumpyPosition, KDLVectorToNumpyArray
from baxterpkg.robot import BaxterRobot
from baxterpkg.cameras import BaxterCamera
from baxterpkg.parameters import BaxterParamsServer
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# ------------------------------------------------------------------------------------------


def getMask(img, binary_threshold=90, filtering_magnitude=5, inverted=False):

    mask = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(float)

    mask[mask < binary_threshold] = 0
    mask[mask >= binary_threshold] = np.max(mask)

    # mask = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((filtering_magnitude, filtering_magnitude)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((int(filtering_magnitude * 1.5), int(filtering_magnitude * 1.5))))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((int(filtering_magnitude * 1.5), int(filtering_magnitude * 1.5))))

    # invert mask
    if inverted:
        mask = np.max(mask) - mask

    return mask


def imagePointCoords(image_frame, binary_threshold=90, filtering_magnitude=5):

    frame = image_frame.copy()

    mask = getMask(frame, binary_threshold=binary_threshold, filtering_magnitude=filtering_magnitude, inverted=False)

    black_pixels = np.where(mask == 0)
    black_pixels = np.flip(black_pixels, axis=0)

    if black_pixels.size == 0:
        return None, frame, mask

    center = np.mean(black_pixels, axis=1).astype(int)

    if center[0] not in range(0, frame.shape[1]) or center[1] not in range(0, frame.shape[0]):
        return None, frame, mask

    cv2.circle(frame, (center[0], center[1]), 5, (0, 0, 255), -1)

    return center, frame, mask


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

# PARAMETERS
CAMERA_CALIBRATION_FILE = "camera_calibration.json"
# object plane coefficients
TABLE_HEIGHT = -0.175  # fixed z value
GRIPPER_HEIGHT = -0.185
PLANE_COEFFS = [0, 0, -1, TABLE_HEIGHT]
# default poses
TABLE_GRASP_HEIGHT = -0.23
TABLE_VIEW_HEIGHT = 0
TABLE_TOP_POSE = Pose(position=Point(x=0.45,
                                     y=0.20,  # left
                                     z=TABLE_VIEW_HEIGHT),
                      orientation=Quaternion(x=1.0,
                                             y=0.0,
                                             z=0.0,
                                             w=0.0))

# ROS node
node = RosNode("openloopvs")
node.setHz(node.setupParameter("hz", 20))


# CAMERA CLASS
cameraparasm = BaxterParamsServer(CAMERA_CALIBRATION_FILE)
leftcameraparams = cameraparasm.getParam("left")

camera = BaxterCamera(node,
                      limb=BaxterCamera.LEFT,
                      camera_matrix=np.array(leftcameraparams["camera_matrix"]))
camera.setting(resolution=(960, 600), exposure=50, gain=50)

# ROBOT CLASS
arm = BaxterRobot(node, limb=BaxterRobot.LEFT)

# robot reset
key = ""  # raw_input(".............. reset? [y]/n: ")
if str(key) not in ["n", "N", "no", "NO"]:
    time.sleep(1)
    arm.setting(max_joint_vel=0.3)
    arm.reset(TABLE_TOP_POSE)
    print("RESET")

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ MAIN LOOP ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

target_frame = None

while node.isActive():

    # hand camera image
    imgr = camera.getimage()

    # gripper current frame
    current_frame = arm.gettf(frame_id=arm.lns + "_gripper")

    if imgr is not None and current_frame is not None:

        # distance target-current frame
        if (target_frame and current_frame) is not None:
            dist = np.linalg.norm(KDLVectorToNumpyArray(target_frame.p - current_frame.p))

        # remove image margin
        marx = 180
        mary = 50
        cv2.rectangle(imgr, pt1=(0, 0), pt2=(imgr.shape[1], marx), color=(255, 255, 255), thickness=-1)  # BUG REMOVE GRIPPER FROM VIEW

        # find target point coordinates (wrt image rf)
        point, output_imgr, mask = imagePointCoords(imgr, binary_threshold=100,   filtering_magnitude=5)
        cv2.circle(output_imgr, (int(output_imgr.shape[1] / 2), int(output_imgr.shape[0] / 2)), 5, (255, 0, 0), -1)
        cv2.circle(output_imgr,  (int(output_imgr.shape[1] / 2), int(output_imgr.shape[0] / 2)), 25, (255, 0, 0), 1)

        cv2.imshow("camera", output_imgr)
        cv2.imshow("mask", mask)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("a"):  # PRESS 'a' to RESET
            arm.reset(TABLE_TOP_POSE)
            continue

        if point is None:
            print("No Point")
            continue

        # vs target
        target3d = camera.px2world(u=point[0], v=point[1],
                                   plane_coefficients=PLANE_COEFFS)
        if target3d is None:
            continue

        xd, yd, zd = target3d

        if key == ord("q"):  # PRESS 'q' to EXIT
            cv2.destroyAllWindows()
            break

        if key == ord("m"):  # PRESS 'm' to MOVE

            # vs target frame wrt gripper rf
            target_frame = PyKDL.Frame()
            target_frame.p = PyKDL.Vector(xd, yd, GRIPPER_HEIGHT)
            target_frame.M = current_frame.M
            arm.movep(target_frame, raw=False)

    arm.refresh()  # refresh tf broadcasting and topic publishing

    node.tick()
