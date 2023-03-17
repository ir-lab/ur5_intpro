#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from ur5_intpro.msg import Ur5Joints, CamPose, TeleopPose, Spacemouse
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints
from std_msgs.msg import String
import threading
import time
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2mat, euler2quat, mat2euler, quat2euler
from transforms3d.quaternions import quat2mat, mat2quat,qmult, qinverse
from transforms3d.affines import compose
import numpy as np
import pyspacemouse
import cv2

from typing import *

class ROS2Unity:
    def __init__(self) -> None:
        rospy.init_node("ros2unity")
        self.cmd_pub = rospy.Publisher("/real_ur5/joints",Ur5Joints,queue_size=1)
        self.pov_pub = rospy.Publisher("/pov_pose",CamPose,queue_size=1)
        self.obj_pub = rospy.Publisher("/obj_pose",TeleopPose,queue_size=1)
        self.sm_pub = rospy.Publisher("/spacemouse_pose",Spacemouse,queue_size=1)
        self.sw_pub = rospy.Publisher("/smartwatch_pose",TeleopPose,queue_size=1)
        self.ur5_joints = [0,0,0,0,0,0]
        self.pov_pose   = [0,0,0,0,0,0,1]
        self.obj_pose   = [0,0,0,0,0,0,1]
        self.origin_pose   = [0,0,0,0,0,0,1]
        self.obj_pose   = [0,0,0,0,0,0,1]
        self.sm_msg = Spacemouse()
        self.primitive_msg  = String()
        # smartwatch pose is normalized wrt arm length 
        # unnormalized using constraints of unity workspace
        self.unity_xmin = -0.75
        self.unity_xmax =  0.75
        self.unity_zmin =  0.20
        self.unity_zmax =  1.00
        self.unity_ymin =  0.35
        self.unity_ymax =  0.75
        self.smartwatch_stream = [0] * 7
        self.smartwatch_pose = TeleopPose()
        self.read_success = False
        self.sm_state = None
        print("sleeping for 1 second....")
        rospy.sleep(1)

        rospy.Subscriber("/ur5/joints", ur5Joints, self.joints_callback)
        rospy.Subscriber("/unity_primitive_id", String, self.unity_primitive_callback)
        rospy.Subscriber("/smartwatch_stream", Float32MultiArray, self.smartwatch_callback)
        # rospy.Subscriber("/vrpn_client_node/POV/pose", PoseStamped, self.pov_callback)
        rospy.Subscriber("/vrpn_client_node/HandLeft/pose", PoseStamped, self.pov_callback)
        rospy.Subscriber("/vrpn_client_node/HandRight/pose", PoseStamped, self.hand_callback)
        # rospy.Subscriber("/vrpn_client_node/Origin/pose", PoseStamped, self.orgin_callback)
        
    def open_sm(self):
        self.read_success = pyspacemouse.open()
        
    def joints_callback(self,msg:ur5Joints):
        for idx, p in enumerate(msg.positions):
            self.ur5_joints[idx] = p

    def unity_primitive_callback(self,msg):
        primitive = msg.data
        image = np.zeros((300,500,3),dtype=np.uint8)
        image = cv2.putText(image, "primitive:", (50,70),
                            cv2.FONT_HERSHEY_COMPLEX,
                            2.5, (255,255,255), 3)
        image = cv2.putText(image, primitive, (110,200),
                            cv2.FONT_HERSHEY_PLAIN,
                            5, (0,0,255), 4)
        cv2.imshow("primitive", image)
        cv2.waitKey(1)
        
    def pov_callback(self,msg:PoseStamped):
        self.pov_pose[0]= msg.pose.position.x
        self.pov_pose[1]= msg.pose.position.y
        self.pov_pose[2]= msg.pose.position.z
        self.pov_pose[3]= msg.pose.orientation.w
        self.pov_pose[4]= msg.pose.orientation.x
        self.pov_pose[5]= msg.pose.orientation.y
        self.pov_pose[6]= msg.pose.orientation.z

    def hand_callback(self,msg:PoseStamped):
        self.obj_pose[0]= msg.pose.position.x
        self.obj_pose[1]= msg.pose.position.y
        self.obj_pose[2]= msg.pose.position.z
        self.obj_pose[3]= msg.pose.orientation.w
        self.obj_pose[4]= msg.pose.orientation.x
        self.obj_pose[5]= msg.pose.orientation.y
        self.obj_pose[6]= msg.pose.orientation.z

    def orgin_callback(self,msg:PoseStamped):
        self.origin_pose[0]= msg.pose.position.x
        self.origin_pose[1]= msg.pose.position.y
        self.origin_pose[2]= msg.pose.position.z
        self.origin_pose[3]= msg.pose.orientation.w
        self.origin_pose[4]= msg.pose.orientation.x
        self.origin_pose[5]= msg.pose.orientation.y
        self.origin_pose[6]= msg.pose.orientation.z
    
    
    def smartwatch_callback(self,msg:Float32MultiArray):
        self.smartwatch_stream = msg.data[0:7]
        
    def convert_tf(self,tf_mat:np.ndarray, msg:Any, cord= "unity"):
        # need condition of switch between unity and optitrack cordinates
        # manually changing coordinate frame to match unity coordinate frame
        msg.x  = -tf_mat[0,-1]
        msg.y  =  tf_mat[1,-1] - 0.25
        msg.z  =  tf_mat[2,-1] - 0.25
        _q = mat2quat(tf_mat[0:3,0:3])
        msg.x_ = -_q[1]
        msg.y_ = _q[2]
        msg.z_ = _q[3]
        msg.w_ = -_q[0]
        return msg
    
    
    def unnormalizer_smarwatchdata(self,xyz:list):
        x,y,z = (*xyz,)
        # x = -(self.unity_xmax-self.unity_xmin)  * x * 0.8
        # y = (self.unity_ymax-self.unity_ymin)   * y * 1.5
        # z = -(self.unity_zmax-self.unity_zmin)  * z * 0.5
        
        return [-x,y,z]

    def spacemouse_publish(self):
        while not rospy.is_shutdown():
            self.sm_state = pyspacemouse.read()
            if self.sm_state is not None and self.read_success:
                self.sm_msg.xyz = [-self.sm_state.x,self.sm_state.z, -self.sm_state.y]
                self.sm_msg.rpy = [-self.sm_state.pitch, self.sm_state.yaw, -self.sm_state.roll]
                self.sm_msg.lbutton = bool(self.sm_state.buttons[0])
                self.sm_msg.rbutton = bool(self.sm_state.buttons[1])
                self.sm_pub.publish(self.sm_msg)      
            rospy.sleep(1/120)  
            
                   
    def ros2unity_ur5(self):
        rel_tf_pov  = compose(self.pov_pose[0:3], quat2mat(self.pov_pose[3:]), [1,1,1])
        rel_tf_obj = compose(self.obj_pose[0:3], quat2mat(self.obj_pose[3:]), [1,1,1])
        
        smartwatch_xyz = self.unnormalizer_smarwatchdata(self.smartwatch_stream[:3])
        smartwatch_quat = self.smartwatch_stream[3:]
        smartwatch_pose = TeleopPose()
        smartwatch_pose.x = smartwatch_xyz[0]
        smartwatch_pose.y = smartwatch_xyz[1]
        smartwatch_pose.z = smartwatch_xyz[2]
        smartwatch_pose.x_ = smartwatch_quat[1]
        smartwatch_pose.y_ = smartwatch_quat[2]
        smartwatch_pose.z_ = smartwatch_quat[3]
        smartwatch_pose.w_ = smartwatch_quat[0]
           
        
        self.pov_pub.publish(self.convert_tf(rel_tf_pov, CamPose()   , cord = "opti" ))
        self.obj_pub.publish(self.convert_tf(rel_tf_obj, TeleopPose(), cord = "opti"))
        self.sw_pub.publish(smartwatch_pose)
        

    def ros2unity_pov(self):
        msg = Ur5Joints()
        msg.joints = self.ur5_joints     
        self.cmd_pub.publish(msg)

if __name__ == '__main__':
        
   
   r2u = ROS2Unity()
   r2u.open_sm()
   th = threading.Thread(target=r2u.spacemouse_publish,args=())
   th.start()
   while not rospy.is_shutdown():
       try:
           r2u.ros2unity_ur5()
           r2u.ros2unity_pov()
           rospy.sleep(1/120)
       except Exception as e:
           print("main loop error ", e)




