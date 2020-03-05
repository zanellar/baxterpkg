#!/usr/bin/env python

import cv2
import argparse
import PyKDL
import numpy as np
import time
import math

from superros.comm import RosNode
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


# ------------------------------------------------------------------------------------------

# PARAMETERS
CAMERA_CALIBRATION_FILE = "camera_calibration.json"

kp_xy = 0.00005
kp_rot = 0.01
top_table_pose = Pose(position=Point(x=0.60,
                                     y=-0.45,
                                     z=-0.05),
                      orientation=Quaternion(x=1.0,
                                             y=0.0,
                                             z=0.0,
                                             w=0.0))

# ROS node
node = RosNode("dloshaping")
node.setHz(node.setupParameter("hz", 50))

# CAMERA CLASS
cameraparasm = BaxterParamsServer(CAMERA_CALIBRATION_FILE)
leftcameraparams = cameraparasm.getParam("right")

camera_right = BaxterCamera(node,
                            limb=BaxterCamera.RIGHT,
                            camera_matrix=np.array(leftcameraparams["camera_matrix"]))
camera_right.setting(resolution=(960, 600), exposure=100, gain=40)

# ROBOT CLASS
arm_right = BaxterRobot(node, limb=BaxterRobot.RIGHT)

# robot reset
key = raw_input(".............. reset? [y]/n: ")
if str(key) not in ["n", "N", "no", "NO"]:
    arm_right.setting(max_joint_vel=0.3)
    arm_right.reset(top_table_pose)
    arm_right.setting(max_joint_vel=0.001)
    print("RESET")

################################################
################### MAIN LOOP ##################
################################################

go = False
while node.isActive():

    # get wrist image
    imgr = camera_right.getimage()

    if imgr is not None:

        ################### IMAGE PROCESSING ##################
        # remove image margin
        marx = 180
        mary = 50
        cv2.rectangle(imgr, pt1=(0, 0), pt2=(imgr.shape[1], marx), color=(255, 255, 255), thickness=-1)  # BUG REMOVE GRIPPER FROM VIEW
        # cv2.rectangle(imgr, pt1=(0, marx), pt2=(mary, imgr.shape[0]), color=(255, 255, 255), thickness=-1)
        # cv2.rectangle(imgr, pt1=(imgr.shape[1] - mary,  marx), pt2=(imgr.shape[1], imgr.shape[0]), color=(255, 255, 255), thickness=-1)

        # find target point coordinates (wrt image rf)
        point, output_imgr, mask = imagePointCoords(imgr, binary_threshold=30,   filtering_magnitude=6)

        cv2.imshow("camera", output_imgr)
        cv2.imshow("mask", mask)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # PRESS 'q' to EXIT
            cv2.destroyAllWindows()
            break
        if key == ord("m"):  # PRESS 'm' to MOVE
            go = True
        if key == ord("a"):  # PRESS 'a' to RESET
            arm_right.reset(top_table_pose)

        if point is None:
            print("No Point")
            continue

        ###################### CONTROL ########################
        imgr_center = np.array(imgr.shape[0:2]) / 2
        imgr_center = np.flip(imgr_center)
        ctrl_x, ctrl_y = -kp_xy * (point - imgr_center)
        print("%8.4f, %8.4f" % (ctrl_x, ctrl_y))

        # translation vector (wrt image rf)
        tr_ctrl = PyKDL.Vector(ctrl_x, ctrl_y, 0)

        # rotation from image rf to camera rf
        R_img2cam = PyKDL.Rotation().RotZ(math.pi)
        frame_cam2eef = arm_right.gettf(frame_id="right_hand_camera",
                                        parent_id="right_gripper")
        # rotation from camera rf to gripper rf
        R_cam2eef = frame_cam2eef.M

        # translation vector (wrt gripper rf)
        tr_ctrl = R_cam2eef * R_img2cam * tr_ctrl

        # target frame  (wrt gripper rf)
        frame_ctrl = PyKDL.Frame()
        frame_ctrl.p = tr_ctrl

        # transformation frame from gripper to base
        frame_eef2base = arm_right.gettf(frame_id="right_gripper")

        # TARGET FRAME  (wrt base rf)
        frame_ctrl = frame_eef2base * frame_ctrl

        print("@@@@@@", R_img2cam * tr_ctrl)

        ######################### MOTION ######################
        if go:
            arm_right.movep(frame_ctrl)
            go = False

        # broadcast target tf
        arm_right._broadcastTargetFrame(frame_ctrl)

    # # refresh tf broadcasting and topic publishing
    # arm_right.refresh()

    node.tick()
