#!/usr/bin/env python

import cv2
import argparse
import PyKDL
import numpy as np
import time
import math

from superros.comm import RosNode
from superros.transformations import KLDFrameToNumpyRotation, KLDFrameToNumpyPosition
from baxterpkg.robot import BaxterMoveit
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


def image2world(u, v, camera_matrix, tf_camera_base, plane_coefficients):
    if tf_camera_base is None:
        return None

    # camera frame wrt base frame
    camera_rotation_matrix = KLDFrameToNumpyRotation(tf_camera_base)
    camera_position_vector = KLDFrameToNumpyPosition(tf_camera_base)

    # from image point [pixels] to cartasian ray [meters]
    image_point = np.array([
        u,
        v,
        1.0
    ]).reshape(3, 1)
    ray = np.matmul(np.linalg.inv(camera_matrix), image_point)
    ray = ray / np.linalg.norm(ray)
    ray = ray.reshape(3)

    # intersection cartesian ray and target plane
    camera_frame = np.concatenate((camera_rotation_matrix, camera_position_vector.reshape(3, 1)), axis=1)
    camera_frame_h = np.concatenate((camera_frame, [[0, 0, 0, 1]]))
    plane_coefficients = np.matmul(np.matrix.transpose(camera_frame_h), plane_coefficients)
    t = -(plane_coefficients[3]) / (
        plane_coefficients[0] * ray[0] +
        plane_coefficients[1] * ray[1] +
        plane_coefficients[2] * ray[2]
    )
    x = ray[0] * t
    y = ray[1] * t
    z = ray[2] * t
    plane_point = np.array([x, y, z]) + np.array([0.065, 0.035, 0])  # BUG offset camera extrinsics @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    # point with respect to the base frame
    plane_point_h = np.concatenate((plane_point, [1]))
    plane_point_h = np.matmul(camera_frame_h, plane_point_h)
    plane_point = plane_point_h[:3].reshape(3)

    return plane_point


# ------------------------------------------------------------------------------------------

# PARAMETERS
CAMERA_CALIBRATION_FILE = "camera_calibration.json"
# object plane coefficients
TABLE_HEIGHT = -0.2075  # fixed z value
GRIPPER_HEIGHT = -0.19
PLANE_COEFFS = [0, 0, -1, TABLE_HEIGHT]
# default poses
TABLE_TOP_POSE = Pose(position=Point(x=0.60,
                                     y=-0.45,
                                     z=-0.05),
                      orientation=Quaternion(x=1.0,
                                             y=0.0,
                                             z=0.0,
                                             w=0.0))

# ROS node
node = RosNode("openloopvs")
node.setHz(node.setupParameter("hz", 30))

# CAMERA CLASS
cameraparasm = BaxterParamsServer(CAMERA_CALIBRATION_FILE)
leftcameraparams = cameraparasm.getParam("right")

camera_right = BaxterCamera(node,
                            limb=BaxterCamera.RIGHT,
                            camera_matrix=np.array(leftcameraparams["camera_matrix"]))
camera_right.setting(resolution=(960, 600), exposure=30, gain=60)

# ROBOT CLASS
arm_right = BaxterMoveit(node, limb=BaxterMoveit.RIGHT)
arm_right.scene.remove_world_object("table")
time.sleep(1)
arm_right.obstacle("table", position=[0.7, 0, -0.563], size=[1, 2, 0.705])
time.sleep(1)

# robot reset
key = raw_input(".............. reset? [y]/n: ")
if str(key) not in ["n", "N", "no", "NO"]:
    # arm_right.setting(max_joint_vel=0.3)
    arm_right.movep(TABLE_TOP_POSE)
    # arm_right.setting(max_joint_vel=0.001)
    print("RESET")

# ################################################
# ################### MAIN LOOP ##################
# ################################################

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
        point, output_imgr, mask = imagePointCoords(imgr, binary_threshold=45,   filtering_magnitude=5)
        print("point", point)
        cv2.imshow("camera", output_imgr)
        cv2.imshow("mask", mask)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # PRESS 'q' to EXIT
            cv2.destroyAllWindows()
            break
        if key == ord("m"):  # PRESS 'm' to MOVE
            go = True
        if key == ord("a"):  # PRESS 'a' to RESET
            arm_right.reset(TABLE_TOP_POSE)

        if point is None:
            print("No Point")
            continue

        ###################### CONTROL ########################
        camera_frame = camera_right.getCameraFrame()
        target3d = image2world(u=point[0], v=point[1],
                               camera_matrix=camera_right.camera_matrix,
                               tf_camera_base=camera_frame,
                               plane_coefficients=PLANE_COEFFS)
        if target3d is None:
            continue

        xd, yd, zd = target3d
        print("%8.4f, %8.4f, %8.4f" % (target3d[0], target3d[1], target3d[2]))

        current_frame = arm_right.gettf(frame_id="right_gripper")

        # target frame  (wrt gripper rf)
        target_frame = PyKDL.Frame()
        target_frame.p = PyKDL.Vector(xd, yd, GRIPPER_HEIGHT)
        target_frame.M = current_frame.M

        ######################### MOTION ######################
        if go:
            arm_right.showplan(target_frame)
            arm_right.movep(target_frame, raw=False)
            go = False

        # broadcast target tf
        arm_right._broadcastTargetFrame(target_frame)

    # # refresh tf broadcasting and topic publishing
    # arm_right.refresh()

    node.tick()
