#!/usr/bin/env python

import cv2
import argparse
import numpy as np
import PyKDL
import time
from superros.comm import RosNode
import superros.transformations as transformations

# from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from cv_bridge import CvBridge, CvBridgeError

import baxter_interface
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


class BaxterRobot(object):
    LEFT = -1
    RIGHT = 1

    def __init__(self, node, limb=+1):

        self.node = node
        self.lns = "right" if limb == BaxterRobot.RIGHT else "left"

        # Inverse Kinemaics
        servicename = "ExternalTools/" + self.lns + "/PositionKinematicsNode/IKService"
        self.iksvc = self.node.getServiceProxy(servicename, SolvePositionIK)
        self.ikreq = SolvePositionIKRequest()

        # Motion
        while True:
            try:
                self.limb = baxter_interface.Limb(self.lns)
            except Exception as e:
                print(e)
                continue
            else:
                break

        self.targetframe = None
        self.currentframe = None

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        print("Enabling robot... ")
        self._rs.enable()

        # Gripper
        self._gripper = baxter_interface.Gripper(self.lns)
        if not self._gripper.calibrated():
            self._gripper.calibrate()

    def _broadcastTargetFrame(self, tf=None, frame_id="target_tf", parent_id='base'):
        if tf is not None:
            if type(tf) is Pose:
                tf = transformations.PoseToKDL(tf)
            self.node.broadcastTransform(tf,
                                         frame_id,
                                         parent_id,
                                         self.node.getCurrentTime())
            self.targetframe = tf

    def ik(self, pose):
        if type(pose) is PyKDL.Frame:
            pose = transformations.KDLToPose(pose)

        # https://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service
        hdr = Header(stamp=self.node.getCurrentTime(), frame_id='base')
        # self.ikreq.seed_mode = self.ikreq.SEED_CURRENT
        self.ikreq.pose_stamp = [PoseStamped(header=hdr, pose=pose)]
        if self.iksvc is not None:
            try:
                res = self.iksvc(self.ikreq)
            except Exception as e:
                print("Service call failed: %s" % (e,))
                return None
            joints = dict(zip(res.joints[0].name, res.joints[0].position))
            return joints
        else:
            return None

    def setting(self, max_joint_vel=0.3):
        self.limb.set_joint_position_speed(max_joint_vel)
        self.movej(self.getjoints(), raw=False)
        time.sleep(1)

    def reset(self, pose=None):
        self.gripperOpen()
        self.targetframe = pose
        if pose is None:
            self.limb.move_to_neutral()
        else:
            self.movep(pose, raw=False)

    def refresh(self):
        self._broadcastTargetFrame(self.targetframe)

    def gettf(self, frame_id=None, parent_id='base', wait=50):
        if frame_id is None:
            frame_id = self.lns + '_gripper'

        tf = None
        while tf is None and wait > 0:
            wait -= 1
            try:
                tf = self.node.retrieveTransform(frame_id=frame_id,
                                                 parent_frame_id=parent_id,
                                                 time=-1)
            except Exception as e:
                tf = None
                print(e)

        return tf

    def getjoints(self):
        return self.limb.joint_angles()

    def movej(self, q, raw=True):
        ''' move in joint space by giving a joint configuration as dictionary'''
        if raw:
            self.limb.set_joint_positions(q, raw=raw)
        else:

            self.limb.move_to_joint_positions(q)

    def movep(self, pose, raw=True):
        ''' move the eef in Cartesian space by giving a geometry_msgs.Pose or a PyKDL.Frame'''

        if type(pose) is PyKDL.Frame:
            self.targetframe = pose
            pose = transformations.KDLToPose(pose)
        elif type(pose) is Pose:
            self.targetframe = transformations.PoseToKDL(pose)

        q = self.ik(pose)
        if q is not None:
            self.movej(q, raw=raw)
        else:
            print("\nNO SOLUTION TO IK\n" * 20)
        self._broadcastTargetFrame(pose)

        return q

    def done(self, tol=0.1, mode=0):
        ''' @mode 
        0: distance between origins of current and target frame smaller than @tol
        1: (NOT IMPLEMENTED) distance between current and target joints smaller than @tol
        2: (NOT IMPLEMENTED) distance(?) between current and target frames smaller than @tol '''

        if mode == 1:
            print("NOT IMPLEMENTED")
        elif mode == 2:
            print("NOT IMPLEMENTED")
        else:
            self.currentframe = self.gettf(self.lns + '_gripper')
            dist = np.linalg.norm(transformations.KDLVectorToNumpyArray(self.targetframe.p - self.currentframe.p))
            print(dist)
            if dist <= tol:
                return True
            else:
                return False

    def speedj(self, dq):
        ''' move joints in velocity  '''
        self.limb.set_joint_velocities(dq)

    def speedp(self, dp):
        ''' move eef in velocity  '''
        # TODO not tested!!!!
        # if type(dp) is PyKDL.Frame:
        #     dp = transformations.KDLToPose(dp)
        # dq = self.ik(dp)
        # self.speedj(dq)

    def gripperOpen(self, waitsec=1):
        self._gripper.open()
        time.sleep(waitsec)

    def gripperClose(self, waitsec=1):
        self._gripper.close()
        time.sleep(waitsec)


from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
import moveit_msgs.msg


class BaxterMoveit(BaxterRobot):
    LEFT = -1
    RIGHT = 1

    def __init__(self, node, limb=+1):
        super(BaxterMoveit, self).__init__(node, limb=+1)
        self.group = MoveGroupCommander(self.lns + "_arm")
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self._info()

    def _info(self):
        '''Getting Basic Information'''
        # name of the reference frame for this robot:
        print("@@@@@@@@@@@@ Reference frame: %s" % self.group.get_planning_frame())

        # We can also print the name of the end-effector link for this group:
        print("@@@@@@@@@@@@ End effector: %s" % self.group.get_end_effector_link())

        # We can get a list of all the groups in the robot:
        print("@@@@@@@@@@@@ Robot Groups: @@@@@@@@@@@@@", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("@@@@@@@@@@@@ Printing robot state @@@@@@@@@@@@@")
        print(self.robot.get_current_state())

    def obstacle(self, name, position, size):
        planning_frame = self.robot.get_planning_frame()
        pose = PoseStamped()
        pose.header.frame_id = planning_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        self.scene.add_box(name, pose, tuple(size))

    def movej(self, q, raw=False):
        ''' move in joint space by giving a joint configuration as dictionary'''
        if raw:
            print("in moveit 'raw' motion is not avaiable")
        # succ = False
        # while succ is False:
        succ = self.group.go(q, wait=True)
        # self.group.stop()  # ensures that there is no residual movement

    def showplan(self, target):
        if type(target) is PyKDL.Frame or type(target) is Pose:
            q = self.ik(target)
        elif type(target) is dict:
            q = target
        else:
            print("Target format error")
            return
        self.group.set_joint_value_target(q)
        self.group.plan()

    def movep(self, pose, raw=False):
        ''' move the eef in Cartesian space by giving a geometry_msgs.Pose or a PyKDL.Frame'''
        if type(pose) is PyKDL.Frame:
            pose = transformations.KDLToPose(pose)

        q = self.ik(pose)
        if q is not None:
            self.movej(q, raw=raw)
        else:
            print("\nNO SOLUTION TO IK\n" * 20)
