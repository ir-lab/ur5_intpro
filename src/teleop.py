#!/usr/bin/env python3

import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl, ur5Tool
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, Float32MultiArray
from ur5_intpro.msg import Ur5Joints as unityUr5
from transforms3d.affines import compose
from transforms3d.euler import mat2euler, euler2mat
from transforms3d.quaternions import quat2mat , mat2quat


import threading
import os 
import sys
import time

from robot_control import *


# Joy message axes mappings for Dualshock 4 controller
LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 3
RIGHT_Y = 4
DPAD_X = 6
DPAD_Y = 7
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

# Buttons
X_BUTTON = 0
CIRCLE_BUTTON = 1
TRIANGLE_BUTTON = 2
SQUARE_BUTTON = 3
L1_BUTTON = 4
R1_BUTTON = 5
L2_BUTTON = 6
R2_BUTTON = 7
START = 9
SELECT = 8
L3_BUTTON = 11
R3_BUTTON = 12
PS_BUTTON = 10

# Axis ranges
STICK_MAX_VALUE = 1.0
STICK_MIN_VALUE = -1.0
TRIGGER_MAX_VALUE = 1.0
TRIGGER_MIN_VALUE = -1.0

# Gamepad thumbstick deadzone
STICK_DEADZONE = 0.15

CONTROL_FREQUENCY = 30
NUM_JOINTS = 6
MAX_CARTESIAN_ACCELERATION = 8.0
MIN_CARTESIAN_ACCELERATION = -8.0
MAX_CARTESIAN_VELOCITY = 5.0
MAX_ANGULAR_ACCELERATION = 13.0
MIN_ANGULAR_ACCELERATION = -8.0
MAX_ANGULAR_VELOCITY = 7.0
MAX_RADIUS_ACCELERATION = 8.0
MIN_RADIUS_ACCELERATION = -5.0
MAX_RADIUS_VELOCITY = 2.5


class Robot_Teleop:
    
    
    def __init__(self) -> None:
        rospy.init_node("robot_teleop",anonymous=True)
        self.robot_control = Robot_Control()
        self.joy_msg = Joy()
        self.unity_ur5_msg = unityUr5()
        self.ur5_msg  = ur5Joints()
        # self.gazebo_ur5_joints =  JointState()
        self.gazebo_ur5_joints = [0]*6
        # print("sleeeping for 1 second....")
        # rospy.sleep(1)
        
        self.swdata = Float32MultiArray()
        rospy.Subscriber("/unity_ur5/joints",unityUr5,self.unity_ur5_callback)
        rospy.Subscriber("/ur5/joints",ur5Joints,self.ur5_callback)
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        rospy.Subscriber("/smartwatch_stream",Float32MultiArray,self.smartwatch_callback)
        # rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,buff_size=1)
        
        self.real_ur5_pub         = rospy.Publisher("/ur5/control",ur5Control,queue_size=1)
        self.unity_ur5_pub        = rospy.Publisher("/ur5_goal/joints",unityUr5,queue_size=1)
        self.unity_robotiq_pub    = rospy.Publisher("/robotiq/joint",Float64,queue_size=1)
        # self.gazebo_ur5_publisher = [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
        
        
        self.x     = - 0.258
        self.y     =   0.2864
        self.z     =   0.3034
        self.roll  =   0.5762139261965049  
        self.pitch =   np.pi/2
        self.yaw   =   2.1492796379453947 # - np.pi/2
        
        self.init_cart = True
        
        self.x_max =  0.7
        self.x_min = -0.7
        self.y_max =  0.7
        self.y_min = -0.4
        self.z_max =  0.4
        self.z_min =  0.0 
        self.epsilon = 0.001
        
        self.gripper = 0.0
        self.gripper_min = 0
        self.gripper_max = 45
        self.ur5_flag = False
        
    def smartwatch_callback(self,msg) -> None:
        self.swdata = msg
        
        
    def joy_callback(self,msg) -> None:
        self.joy_msg = msg
        
    def unity_ur5_callback(self,msg) -> None:
        self.unity_ur5_msg = msg
    
    def ur5_callback(self,msg) -> None:
        self.ur5_msg = msg
        
    def joint_state_callback(self,msg) -> None:
        self.gazebo_ur5_joints[0] = msg.position[3]
        self.gazebo_ur5_joints[1] = msg.position[2]
        self.gazebo_ur5_joints[2] = msg.position[0]
        self.gazebo_ur5_joints[3] = msg.position[4]
        self.gazebo_ur5_joints[4] = msg.position[5]
        self.gazebo_ur5_joints[5] = msg.position[6]
         
    def init_xyz(self):
        tf_mat = self.robot_control.get_fk_sol(self.unity_ur5_msg.joints)
        self.x = tf_mat[0,-1]
        self.y = tf_mat[1,-1]
        self.z = tf_mat[2,-1]
        
    def check_thresh(self, var, var_max, var_min) -> float:
        if var > var_max:
            var = var_max
        if var < var_min:
            var = var_min
        return var
        
    def reached_goal(self, c_joints, g_joints, abs_tol = 1e-3, rel_tol = 1e-3):
        if len(c_joints) != len(g_joints):
            raise ValueError("missmatch between goal joints and current joints")
        if np.allclose(c_joints,g_joints, atol=abs_tol, rtol=rel_tol):
            return True
        else:
            return False 
    
    
    def move_sm2ee(self):
        self.x += (-self.swdata.data[0])
        self.y += self.swdata.data[2]
        self.z += self.swdata.data[1]
        
        
        self.x  = self.check_thresh(self.x,self.x_max,self.x_min)
        self.y  = self.check_thresh(self.y,self.y_max,self.y_min)
        self.z  = self.check_thresh(self.z,self.z_max,self.z_min) 
        
        print_msg = f"""watch :=  x:[{self.x}], y:[{self.y}], z:[{self.z}]"""
        print(print_msg)
        
        
        goal_joints = self.robot_control.get_ik_sol(self.unity_ur5_msg.joints,
                                                    [self.x, self.y, self.z],
                                                    [self.roll, self.pitch, self.yaw])
        
        msg = unityUr5()
        msg.joints = goal_joints
        self.unity_ur5_pub.publish(msg)
        
        
    def move_ee(self):     
        # if self.init_cart:
        #     self.init_xyz()
        #     self.init_cart = False
        
        if not self.joy_msg.axes:
            return   
        self.x += (self.epsilon *  self.joy_msg.axes[LEFT_X])
        self.y += (self.epsilon *  self.joy_msg.axes[LEFT_Y])
        self.z += (self.epsilon *  self.joy_msg.axes[RIGHT_Y])
        
        self.x  = self.check_thresh(self.x,self.x_max,self.x_min)
        self.y  = self.check_thresh(self.y,self.y_max,self.y_min)
        self.z  = self.check_thresh(self.z,self.z_max,self.z_min) 
        
        if self.joy_msg.buttons:
            self.gripper += (5 * self.joy_msg.buttons[R1_BUTTON])
            self.gripper -= (5 * self.joy_msg.buttons[L1_BUTTON])
            self.gripper = self.check_thresh(self.gripper,self.gripper_max,self.gripper_min)
            

        goal_joints = self.robot_control.get_ik_sol(self.unity_ur5_msg.joints,
                                                    [self.x, self.y, self.z],
                                                    [self.roll, self.pitch, self.yaw])
        
        
        ur5_goal_joints = self.robot_control.get_ik_sol(self.ur5_msg.positions,
                                                    [self.x, self.y, self.z],
                                                    [self.roll, self.pitch, self.yaw])
        
        
        # gazebo_goal_joints = self.robot_control.get_ik_sol(self.gazebo_ur5_joints,
        #                                                    [self.y, self.x, self.z],
        #                                                    [self.roll, self.pitch, self.yaw])
                                                        
            
        # ur5_control_msg = ur5Control()
        # ur5_control_msg.command = "movej"
        # ur5_control_msg.values = ur5_goal_joints
        # ur5_control_msg.jointcontrol = True
        # ur5_control_msg.time = (1/30)
        # self.real_ur5_pub.publish(ur5_control_msg)
        
        
        # for idx, pub in enumerate(self.gazebo_ur5_publisher):
        #     pub.publish(gazebo_goal_joints[idx])
            
        
        print_msg = f"""cartesian goal:=  x:[{self.x}], y:[{self.y}], z:[{self.z}]\ngripper command:= angle:[{self.gripper}]\n"""
        print(print_msg)
        
        msg = unityUr5()
        msg.joints = goal_joints
        self.unity_ur5_pub.publish(msg)
        
        gmsg = Float64()
        gmsg.data = self.gripper
        self.unity_robotiq_pub.publish(gmsg)
        
    # reset the pid ros paramters 
    def set_pid_param(self, p = 1.0, i = 0.0, d = 0.0):
        rospy.set_param("/ur5/control_params/p_gain",p)
        rospy.set_param("/ur5/control_params/i_gain",i)
        rospy.set_param("/ur5/control_params/d_gain",d)
    
    # def teleop_watch(self):
    #     pass
        
if __name__ == "__main__":
    teleop = Robot_Teleop()
    
    teleop.robot_control.call_home_service(home_joints["rad"],
                                           "/ur5_goal/joints",
                                           0.5,"unity")
    
    # teleop.robot_control.call_home_service(home_joints["rad"],
    #                                        "",
    #                                        0.5,"gazebo")
    # rospy.sleep(0.5)
    
    # teleop.robot_control.call_home_service(home_joints["rad"],
    #                                        "/ur5/control",
    #                                        2,"real")
    
    # rospy.sleep(0.5)
    
    # teleop.set_pid_param( p = 2.0, 
    #                       i = 0.0,
    #                       d = 0.45)
    
    rospy.sleep(1)
    
    while not rospy.is_shutdown():
        print("telop_running")
        # teleop.move_ee()
        teleop.move_sm2ee()
        # teleop.move_unity()
        rospy.sleep(1/30)
        # pass