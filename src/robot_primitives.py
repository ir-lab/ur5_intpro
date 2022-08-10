#! /usr/bin/env python3

from xml.etree.ElementTree import parse
import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl
from sensor_msgs.msg import JointState, Joy
from transforms3d.affines import compose
from transforms3d.euler import mat2euler, euler2mat
from transforms3d.quaternions import quat2mat , mat2quat
import threading
import os
import sys
import time
import threading
import argparse
from six.moves import queue
import serial
from ds4_mapper import DS4_Mapper
from utils import ur5_intpro_utils



class ROBOT_PRIMITIVES:
    
    
    def __init__(self):
        rospy.init_node("ur5_kdl")
        
        
        self.package_path = rospkg.RosPack().get_path("ur5_intpro")
        self.rate = 5
        self.general_params = ur5_intpro_utils.load_yaml(os.path.join(self.package_path,"config","general_params.yaml"))
        
         # init ros msgs
        self.joint_state_msg = JointState()
        self.joy_msg         = Joy()
        self.ur5_control_msg = ur5Control()
        self.r2fg_msg        = gSimpleControl()
        self.ur5_control_msg.command      = rospy.get_param("irl_robot_command",  default="movej") 
        self.ur5_control_msg.acceleration = rospy.get_param("irl_robot_accl",     default=np.pi/2)
        self.ur5_control_msg.velocity     = rospy.get_param("irl_robot_vel",      default=np.pi/2) 
        self.ur5_control_msg.time         = rospy.get_param("irl_robot_com_time", default=5)
        self.ur5_control_msg.jointcontrol = True
        self.ur5_control_timeout = 0
        
        rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,buff_size=1)
        rospy.Subscriber("/ur5/joints",ur5Joints,self.ur5_joints_callback)
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        
        
        
        # kdl setup 
        self.urdf_path     = os.path.join(self.package_path,"urdf",self.general_params["kdl_urdf"])
        self.ur5_tf_chain  = self.init_kdl()
        self.ur5_ik_solver = KDL.ChainIkSolverPos_LMA(self.ur5_tf_chain, 1e-8 ,1000, 1e-6)
        # self.ur5_ik_solver =  KDL.ChainIkSolverPos_NR(Chain=self.ur5_tf_chain)
        
        # ik prameters
        self.goal_x   = self.general_params["ik_goal"]["x"]
        self.goal_y   = self.general_params["ik_goal"]["y"]
        self.goal_z   = self.general_params["ik_goal"]["z"]
        self.yaw      = np.deg2rad(self.general_params["ik_goal"]["yaw"])
        self.pitch    = np.deg2rad(self.general_params["ik_goal"]["pitch"])
        self.roll     = np.deg2rad(self.general_params["ik_goal"]["roll"])
        
        self.sim_ur5_joint_publisher = [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
        self.real_ur5_joint_publisher   = rospy.Publisher("/ur5/control",ur5Control,queue_size=10)
        
        self.goals = self.general_params["goals"]
        self.rosthread = self.rospy_thread()
        print("sleeping for a second.....")
        rospy.sleep(1)
        

    def ur5_joints_callback(self,msg):
        self.ur5_joints = msg
             
    def joint_state_callback(self,msg):
        self.joint_state_msg = msg
    
    def joy_callback(self,msg):
        self.joy_msg = msg 
        
    def init_kdl(self,root_link="base_link",leaf_link="tool0"):
        kdl_tree = kdl_parser_py.urdf.treeFromFile(self.urdf_path)[1]
        return kdl_tree.getChain(root_link,leaf_link)
    
    
    def rospy_thread(self):
        rospy_thread = threading.Thread(target=rospy.spin,args=())
        rospy_thread.start()
        return rospy_thread
    
    def run_thread(self):
        while not rospy.is_shutdown():
            try:
                print("runing main thread")
                # self.get_ik_sol()
                self.ur5_publisher()
                self.goal_x += 0.005
                rospy.sleep(5)
            except Exception as e:
                print(e)
    
    def get_ik_sol(self, enable_rot=True, yaw_offset=0):
        
        # get goal id (only for testing)
        kdl_init_joints = KDL.JntArray(6)
        kdl_init_joints[0] = self.joint_state_msg.position[3] 
        kdl_init_joints[1] = self.joint_state_msg.position[2] 
        kdl_init_joints[2] = self.joint_state_msg.position[0]
        kdl_init_joints[3] = self.joint_state_msg.position[4]
        kdl_init_joints[4] = self.joint_state_msg.position[5]
        kdl_init_joints[5] = self.joint_state_msg.position[6]        
        
        if yaw_offset:
            print("Using yaw offset")
            tf = compose([0,0,0],euler2mat(0,0,yaw_offset),[1,1,1])
            self.z_offset = self.goal_z + 0.15
            goal_tf = compose([self.goal_x,self.goal_y,self.z_offset],euler2mat(self.yaw,self.pitch,self.roll),[1,1,1])
            goal_tf = np.matmul(tf,goal_tf)
            # print(goal_tf[0:3,-1])
            _xyz = KDL.Vector(goal_tf[0,-1],goal_tf[1,-1],goal_tf[2,-1])
            rot = mat2euler(goal_tf[0:3,0:3])
            _rpy = KDL.Rotation().EulerZYX(rot[2],rot[1],rot[0]) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
            
        else:   
            self.z_offset = self.goal_z + 0.15
            
            _xyz               = KDL.Vector(self.goal_x, self.goal_y, self.z_offset)     
            _rpy               = KDL.Rotation().EulerZYX(self.roll, self.pitch, self.yaw) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
        kdl_goal_frame     = KDL.Frame(_rpy,_xyz)
        kdl_goal_joints    = KDL.JntArray(6)
        self.ur5_ik_solver.CartToJnt(kdl_init_joints, kdl_goal_frame, kdl_goal_joints)
        return kdl_goal_joints
        
    def ur5_publisher(self):        
        for g in self.goals:
            self.goal_x = g[0]
            # self.goal_x = 0.2
            self.goal_y = g[1]
            self.goal_z = g[2]
            
            # goal_joints        = self.get_ik_sol(yaw_offset= 0)
            goal_joints        = self.get_ik_sol(yaw_offset=np.pi/4)
            self.ur5_control_msg.values = goal_joints
            for i, gj in enumerate(goal_joints):
                self.sim_ur5_joint_publisher[i].publish(gj)
            self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
            # rospy.sleep(1/self.rate)
            print(f"goal: {self.goal_x, self.goal_y, self.goal_z}")
            rospy.sleep(2)
    def __del__(self):
        pass
    
    

if __name__ == '__main__':
    
    try:
        rp = ROBOT_PRIMITIVES()
        rp.run_thread()
    except Exception as e:
        print(e)