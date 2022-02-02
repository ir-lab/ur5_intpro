#! /usr/bin/env python3

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


class Custom_UR5_KDL:
    
    def __init__(self, pkg_name="ur5_intpro", init_delay=0.1, go_midhome=True):
        rospy.init_node("ur5_kdl")
        self.this_pkg_path = rospkg.RosPack().get_path(pkg_name)
        
        self.joint_state_msg = JointState()
        self.joy_msg      = Joy()
        
        
        # kdl setup 
        self.urdf_path = os.path.join(self.this_pkg_path,"urdf/kdl_custom_ur5.urdf")
        self.ur5_tf_chain = self.init_kdl()
        self.ur5_ik_solver =  KDL.ChainIkSolverPos_LMA(self.ur5_tf_chain)
        # ik prameters
        self.goal_x   = 0.4
        self.goal_y   = 0.0
        self.goal_z   = 0.4
        
        # for vertical ee config keep yaw=0.0 pitch=np.pi and roll=0.0
        # for horizontal ee  config keep yaw=np.pi/2.0 pitch=0.0 roll=np.pi/2.0
        self.yaw      = np.pi/2.
        self.pitch    = 0
        self.roll     = np.pi/2.0
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
        self.ur5_publisher(joints=midhome_joints, delay=0.1)
        rospy.sleep(1)
        return True
    
    def get_ik(self, nums_joints=6, enable_rot=False):
        kdl_init_joints = KDL.JntArray(nums_joints)
        for i in range(nums_joints):
            kdl_init_joints[i] = self.joint_state_msg.position[i] 
        just_xyz = KDL.Vector(self.goal_x, self.goal_y, self.goal_z)
        just_rpy = KDL.Rotation().EulerZYX(self.roll, self.pitch, self.yaw) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
        kdl_goal_frame = KDL.Frame(just_rpy,just_xyz)
        kdl_goal_joints = KDL.JntArray(nums_joints)
        self.ur5_ik_solver.CartToJnt(kdl_init_joints, kdl_goal_frame, kdl_goal_joints)
        return kdl_goal_joints
        
    def ur5_publisher(self, joints, delay=0.1):
        for i, publisher in enumerate(self.ur5_joint_publisher):
            publisher.publish(joints[i])
        return 
    
if __name__ == '__main__':
    
    ur5_kdl = Custom_UR5_KDL(go_midhome=False)
    rate    = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            print("running node")  
            kdl_goal_joints = ur5_kdl.get_ik(enable_rot=True)
            ur5_kdl.ur5_publisher(joints=kdl_goal_joints, delay=0.0)
            rate.sleep()
        except KeyboardInterrupt:
            print("shutting down!")