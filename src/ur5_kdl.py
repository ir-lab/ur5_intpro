#! /usr/bin/env python

from xml.etree.ElementTree import parse
import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import JointState, Joy
import threading
import os
import sys
import time
import threading
import argparse
from six.moves import queue
import serial
from ds4_mapper import DS4_Mapper


class Custom_UR5_KDL:
    
    def __init__(self, pkg_name="ur5_intpro", init_delay=0.1, go_midhome=True):
        rospy.init_node("ur5_kdl")
        self.this_pkg_path = rospkg.RosPack().get_path(pkg_name)
        
        self.joint_state_msg = JointState()
        self.joy_msg      = Joy()
        
        # ds4 key mapper
        self.ds4_mapper = DS4_Mapper()
        self.enable_ds4 = False
        self.limit_cart = 0.8
        self.cart_step  = 0.005
        
        # kdl setup 
        self.urdf_path = os.path.join(self.this_pkg_path,"urdf/kdl_custom_ur5.urdf")
        self.ur5_tf_chain = self.init_kdl()
        self.ur5_ik_solver =  KDL.ChainIkSolverPos_LMA(self.ur5_tf_chain, 1e-8 ,1000, 1e-6)
        # self.ur5_ik_solver =  KDL.ChainIkSolverPos_NR(Chain=self.ur5_tf_chain)
        
        # ik prameters

        self.goal_x   = 0.4
        self.goal_y   = 0.0
        self.goal_z   = 0.6
        # for vertical ee config keep yaw=0.0 pitch=np.pi and roll=0.0
        # for horizontal ee  config keep yaw=np.pi/2.0 pitch=0.0 roll=np.pi/2.0
        self.yaw      = 0
        self.pitch    = np.pi
        self.roll     = 0
        self.rot_cont = True
        
        
        
        self.ur5_joint_publisher = [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
        rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,buff_size=1)
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        rospy.sleep(init_delay)
        
        if go_midhome:
            self.init_midhome()
            
    def joint_state_callback(self,msg):
        self.joint_state_msg = msg
    
    def joy_callback(self,msg):
        self.joy_msg = msg 
         
    def start_ros_spin(self):
        print("Starting ros spin")
        rospy.spin()
    
    def init_kdl(self,root_link="base_link",leaf_link="tool0"):
        kdl_tree = kdl_parser_py.urdf.treeFromFile(self.urdf_path)[1]
        return kdl_tree.getChain(root_link,leaf_link)
    
    def init_midhome(self):
        self.goal_x   = 0.2
        self.goal_y   = 0.0
        self.goal_z   = 0.4
        self.yaw      = np.pi/2.
        self.pitch    = 0
        self.roll     = np.pi/2.0
        self.rot_cont = True
        midhome_joints = self.get_ik()
        # midhome_joints = [0.57988, -0.99270, 2.03353, -0.96921, 1.40923, 1.5766]
        self.ur5_publisher(joints=midhome_joints, delay=0.1)
        rospy.sleep(1)
        return True
    
    def get_ik(self, nums_joints=6, enable_rot=False):
        kdl_init_joints = KDL.JntArray(nums_joints)
        kdl_init_joints[0] = self.joint_state_msg.position[2] 
        kdl_init_joints[1] = self.joint_state_msg.position[1] 
        kdl_init_joints[2] = self.joint_state_msg.position[0]
        kdl_init_joints[3] = self.joint_state_msg.position[3]
        kdl_init_joints[4] = self.joint_state_msg.position[4]
        kdl_init_joints[5] = self.joint_state_msg.position[5] 
        just_xyz = KDL.Vector(self.goal_x, self.goal_y, self.goal_z)
        just_rpy = KDL.Rotation().EulerZYX(self.roll, self.pitch, self.yaw) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
        kdl_goal_frame = KDL.Frame(just_rpy,just_xyz)
        kdl_goal_joints = KDL.JntArray(nums_joints)
        self.ur5_ik_solver.CartToJnt(kdl_init_joints, kdl_goal_frame, kdl_goal_joints)
        return kdl_goal_joints
        
    def ur5_publisher(self, joints, delay=0.1):
        # while not rospy.is_shutdown() and not np.allclose(joints,self.joint_state_msg.position,atol=1e-3):
        for i, publisher in enumerate(self.ur5_joint_publisher):
            publisher.publish(joints[i])
        return 
    
    def ds4_ik_mapper(self):
        if self.joy_msg.buttons[self.ds4_mapper.box] == 1:
            self.goal_y +=  self.cart_step
            if self.goal_y > self.limit_cart:
                self.goal_y = self.limit_cart
    
        if self.joy_msg.buttons[self.ds4_mapper.circle] == 1:
            self.goal_y -=  self.cart_step
            if self.goal_y < -self.limit_cart:
                self.goal_y = -self.limit_cart
    
        if self.joy_msg.buttons[self.ds4_mapper.l1] == 1:
            if self.joy_msg.buttons[self.ds4_mapper.triangle] == 1:
                self.goal_z += self.cart_step
                if self.goal_z > self.limit_cart:
                    self.goal_z = self.limit_cart
            
            if self.joy_msg.buttons[self.ds4_mapper.cross] == 1:
                self.goal_z -= self.cart_step
                if self.goal_z < -self.limit_cart:
                    self.goal_z = -self.limit_cart
        else:
            if self.joy_msg.buttons[self.ds4_mapper.triangle] == 1:
                self.goal_x +=  self.cart_step
                if self.goal_x > self.limit_cart:
                    self.goal_x = self.limit_cart
        
            if self.joy_msg.buttons[self.ds4_mapper.cross] == 1:
                self.goal_x -=  self.cart_step
                if self.goal_x < -self.limit_cart:
                    self.goal_x = -self.limit_cart
        
        if self.joy_msg.buttons[self.ds4_mapper.l3] == 1:
            self.roll  = 0.0
            self.pitch = np.pi 
            self.yaw   = 0.0 
        
    def ds4_teleop(self):
        try:
            if self.joy_msg.buttons[self.ds4_mapper.ps] == 1:
                self.enable_ds4 = True
                rospy.sleep(0.5)
            if self.enable_ds4:
                print("\n ==== Enabled ds4 teleop ==== \n")
                self.ds4_ik_mapper()
                if self.joy_msg.buttons[self.ds4_mapper.ps] == 1:
                    self.enable_ds4 = False
                    rospy.sleep(0.5)
            
        except IndexError:
            print("Looks ds4 controller is not on!!")
            
        
    
if __name__ == '__main__':
    
    ur5_kdl = Custom_UR5_KDL(go_midhome=False)
    rate    = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            print("running node")  
            kdl_goal_joints = ur5_kdl.get_ik(enable_rot=True)
            # print([np.rad2deg(angle) for angle in kdl_goal_joints])
            print("cartesian goals: {}".format([ur5_kdl.goal_x, ur5_kdl.goal_y, ur5_kdl.goal_z]))
            ur5_kdl.ur5_publisher(joints=kdl_goal_joints, delay=0.0)
            ur5_kdl.ds4_teleop()
            
            rate.sleep()
        except KeyboardInterrupt:
            print("shutting down!")