#!/usr/bin/env python

import cv2
import argparse
import PyKDL
import numpy as np

from superros.comm import RosNode
import superros.transformations as transformations

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError

import baxter_interface
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from baxter_interface import CameraController


class BaxterCamera(object):
    LEFT = -1
    RIGHT = 1

    def __init__(self, node, limb=+1, camera_matrix=None, camera_extrinsics=None):

        self.node = node
        self.lns = "right" if limb == BaxterCamera.RIGHT else "left"

        self.image = None
        self.camera_matrix = camera_matrix
        self.camera_frame = None
        self.camera_extrinsics = camera_extrinsics

        # Camera Control
        self.cameracontrol = CameraController(self.lns + '_hand_camera')
        self.cameracontrol.resolution = (960, 600)
        self.cameracontrol.open()

        # Camera Topic
        self.cvbridge = CvBridge()
        node.createSubscriber('cameras/' + self.lns + '_hand_camera/image', Image, self._image_callback)

        # we create the subscriver only if we are not recieving any explicit camera matrix
        if camera_matrix is None:
            node.createSubscriber('cameras/' + self.lns + '_hand_camera/camera_info', CameraInfo, self._camera_info_callback)

    def getExtrinsics(self):
        ''' The extrinsics are wrt the hand'''
        # we take them only once or we use those that we defined in the init
        if self.camera_extrinsics is None:
            wait = 50
            tf = None
            while tf is None and wait > 0:
                wait -= 1
                try:
                    tf = self.node.retrieveTransform(frame_id=self.lns + '_hand_camera',
                                                     parent_frame_id=self.lns + '_hand',
                                                     time=-1)
                except Exception as e:
                    tf = None
                    print(e)

            self.camera_extrinsics = tf

        return self.camera_extrinsics

    def camera2gripper(self):
        wait = 50
        tf = None
        while tf is None and wait > 0:
            wait -= 1
            try:
                tf = self.node.retrieveTransform(frame_id=self.lns + '_hand_camera',
                                                 parent_frame_id=self.lns + '_gripper',
                                                 time=-1)
            except Exception as e:
                tf = None
                print(e)

        self.camera_extrinsics = tf

        return tf

    def _camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))

    def _image_callback(self, msg):
        try:
            self.image = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def getCameraFrame(self, wait=9999):
        self.camera_frame = None
        while self.camera_frame is None and wait > 0:
            wait -= 1
            try:
                self.camera_frame = self.node.retrieveTransform(frame_id=self.lns + "_hand_camera",  # TODO "_hand_camera" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                                                parent_frame_id="base",
                                                                time=-1)
            except Exception as e:
                print(e)
        return self.camera_frame

    def getimage(self):
        return self.image

    def setting(self, state="o",  resolution=(960, 600), exposure=CameraController.CONTROL_AUTO, gain=CameraController.CONTROL_AUTO):
        ''' 
        Camera Exposure: range is 0-100
        Camera gain: is 0-79
        '''
        if state == "o":
            self.cameracontrol.open()
        elif state == "c":
            self.cameracontrol.close()

        self.cameracontrol.resolution = resolution
        self.cameracontrol.exposure = exposure
        self.cameracontrol.gain = gain

    def reset(self, pose=None):
        self.setting()

    def px2world(self, u, v, plane_coefficients, frame_camera=None):
        ''' 
        @u,v: pixels coordinates in image plane (wrttop-left corner);
        @plane_coefficients: coefficents of the plane where the object lies (wrt the Base reference frame);
        @frame_camera: camera reference frame wrt the robot Base reference frame.
        '''
        if frame_camera is None:
            tf_camera_base = self.getCameraFrame()
            if tf_camera_base is None:
                return None
        else:
            tf_camera_base = frame_camera

        # camera frame wrt base frame
        camera_rotation_matrix = transformations.KLDFrameToNumpyRotation(tf_camera_base)
        camera_position_vector = transformations.KLDFrameToNumpyPosition(tf_camera_base)

        # from image point [pixels] to cartasian ray [meters]
        image_point = np.array([
            u,
            v,
            1.0
        ]).reshape(3, 1)
        ray = np.matmul(np.linalg.inv(self.camera_matrix), image_point)
        ray = ray / np.linalg.norm(ray)
        ray = ray.reshape(3)

        # intersection cartesian ray and target plane
        camera_frame = np.concatenate((camera_rotation_matrix, camera_position_vector.reshape(3, 1)), axis=1)
        camera_frame_h = np.concatenate((camera_frame, [[0, 0, 0, 1]]))
        # camera_extrinsics = np.array([0.0403169,    0.998361,  -0.0406236, 0.04188,   # TODO my calibration
        #                               -0.999135,   0.0406977,  0.00859053, 0.013019,
        #                               0.0102297,   0.0402421,    0.999138,    0.017091,
        #                               0,            0,          0,          1]).reshape((4, 4))
        # camera_frame_h = np.matmul(camera_frame_h, camera_extrinsics)  # TODO my calibration

        # Transform plane from base frame to camera frame
        # https://math.stackexchange.com/questions/2502857/transform-plane-to-another-coordinate-system
        plane_coefficients = np.matmul(np.matrix.transpose(np.linalg.inv(camera_frame_h)), plane_coefficients)
        t = -(plane_coefficients[3]) / (
            plane_coefficients[0] * ray[0] +
            plane_coefficients[1] * ray[1] +
            plane_coefficients[2] * ray[2]
        )
        x = ray[0] * t
        y = ray[1] * t
        z = ray[2] * t
        plane_point = np.array([x, y, z])  # + np.array([0.105, 0.055, 0])  # BUG offset CAMERA extrinsics @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        # point with respect to the base frame
        plane_point_h = np.concatenate((plane_point, [1]))
        plane_point_h = np.matmul(camera_frame_h, plane_point_h)
        plane_point = plane_point_h[:3].reshape(3)

        return plane_point
