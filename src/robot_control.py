#!/usr/bin/env python3

import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl, ur5Tool


from ur5_intpro.msg import Ur5Joints as unityUr5
from transforms3d.affines import compose
from transforms3d.euler import mat2euler, euler2mat
from transforms3d.quaternions import quat2mat , mat2quat

import threading
import os 
import sys
import time
import inspect

from ur5_intpro.srv import *



home_joints = {"deg":[115.55, -105.80, 99.89, -84.24, -89.98, 25.42],
               "rad":[2.0167279506794475, -1.8465583486100006, 1.743409389817136, -1.4702653618800232, -1.5704472609444977, 0.4436626958569586]}


def execution_time(func):
    """"Decorator to check the execution time for the function"""
    def wrapper_func(*args):
        start = time.time()
        func(*args)
        t_elapsed = time.time() - start 
        print(f"function {func} execution time: {t_elapsed} secs")
    return wrapper_func
    
    
class PID:
    """ Close-Loop PID Controller given Goal and Current Values"""
    def __init__(self, p = 1.0, i = 0.0, d = 0.0, i_max = 3, i_min = -3, cutoff_thresh = 3, use_err_mag = False) -> None:
        self.p = p
        self.i = i
        self.d = d
        self.cutoff_thresh = cutoff_thresh
        self.error = 0.0
        self.i_max = i_max
        self.i_min = i_min 
        self.integral_state = 0.0
        self.differential_state  = 0.0
        self.usr_err_mag = use_err_mag
    
    def apply_pid(self, cur_val = 0.0, goal_val = 0.0) -> float:
        self.error = goal_val - cur_val
        if self.usr_err_mag:
            self.error = np.abs(self.error)
               
        self.integral_state += self.error
        if self.integral_state > self.i_max:
            self.integral_state = self.i_max
        elif self.integral_state < self.i_min:
            self.integral_state = self.i_min
           
        
        control_input = ((self.error * self.p) +
                        (self.integral_state * self.i) + 
                        ((self.error - self.differential_state) * self.d))

        self.differential_state = self.error
        if control_input > self.cutoff_thresh:
            control_input = self.cutoff_thresh
        elif control_input < (-self.cutoff_thresh):
            control_input = (-self.cutoff_thresh)        
        return control_input


class Robot_Control:
    """Provides following functionalities
       1. FK 
       2. IK
    """
    def __init__(self) -> None:
        self.ik_solver, self.fk_solver = self.setup_kdl(base_link = "base_link", 
                                                        leaf_link = "fake_end_effector_link")
    

    def setup_kdl(self,base_link,leaf_link) -> tuple:
        """ one time setting up of kinematic chain using kdl"""
        urdf_path = os.path.join(rospkg.RosPack().get_path("ur5_intpro"),"urdf","custom_ur5.urdf")
        kdl_tree  = kdl_parser_py.urdf.treeFromFile(urdf_path)[1]
        kdl_chain = kdl_tree.getChain(base_link,leaf_link) 
        ik_solver = KDL.ChainIkSolverPos_LMA(kdl_chain, 1e-8 ,1000, 1e-6)
        fk_solver = KDL.ChainFkSolverPos_recursive(kdl_chain)
        return ik_solver, fk_solver
    
    
    def get_ik_sol(self,c_joints,xyz,rpy) -> list:
        """ get inverse kinematic solution given goal xyz (meters), 
            rpy (radian) and current joint angles (radian) """
        kdl_c_joints = KDL.JntArray(6)
        for enum, j in enumerate(c_joints):
            kdl_c_joints[enum] = j
        kdl_xyz = KDL.Vector(*xyz)
        kdl_rpy = KDL.Rotation().EulerZYX(*rpy)
        kdl_g_joints = KDL.JntArray(6)
        g_joints = self.ik_solver.CartToJnt(kdl_c_joints, KDL.Frame(kdl_rpy,kdl_xyz), kdl_g_joints)
        return [gj for gj in kdl_g_joints]

    # @execution_time
    def get_fk_sol(self, joints, segmentNr=-1) -> np.ndarray:
        """ get euler and cartesian frames of end effector 
            from given joint angles"""
        joints_ = KDL.JntArray(6)
        frame   = KDL.Frame()
        for idx,j in enumerate(joints):
            joints_[idx] = j
        self.fk_solver.JntToCart(joints_, frame, segmentNr= segmentNr)
        rot_mat = np.empty((3,3))
        for i in range(3):
            for j in range(3):
                rot_mat[i,j] = frame.M[i,j]
        tf_mat = compose([frame.p[0],frame.p[1],frame.p[2]],rot_mat,[1,1,1])
        return tf_mat
    
    def call_home_service(self, home_joints,  topic_name, wait_time, robot_type) -> bool:
        rospy.wait_for_service("go_home")
        try:
            go_home = rospy.ServiceProxy("go_home",GoHome)
            out = go_home(home_joints,
                          topic_name,
                          wait_time,
                          robot_type)
            return out
        except rospy.ServiceException as e:
            print(f"Go Home Service Call Failed: {e}")
            
if __name__ == '__main__':
    rcontrol = Robot_Control()
    while True:
        # rcontrol.call_home_service( home_joints = home_joints.get("rad"), 
                                    # topic_name  = "/unity_ur5/joints",
                                    # wait_time   = 2.0,
                                    # robot_type  = "unity")
        out = rcontrol.get_fk_sol([2.0167279506794475, -1.8465583486100006, 1.743409389817136, -1.4702653618800232, -1.5704472609444977, 0.4436626958569586])
        print(mat2euler(out[0:3,0:3]))
        rospy.sleep(0.5)