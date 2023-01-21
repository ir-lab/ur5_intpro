#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from ur5_intpro.msg import Ur5Joints, CamPose
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints
import threading
import time
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2mat, euler2quat, mat2euler, quat2euler
from transforms3d.quaternions import quat2mat, mat2quat,qmult, qinverse
from transforms3d.affines import compose
import numpy as np


class ROS2Unity:
    def __init__(self) -> None:
        rospy.init_node("ros2unity")
        self.cmd_pub = rospy.Publisher("/real_ur5/joints",Ur5Joints,queue_size=1)
        self.pov_pub = rospy.Publisher("/pov_pose",CamPose,queue_size=1)
        self.ur5_joints = [0,0,0,0,0,0]
        self.pov_pose   = [0,0,0,0,0,0,1]
        self.obj_pose   = [0,0,0,0,0,0,1]
        print("sleeping for 1 second....")
        rospy.sleep(1)

        rospy.Subscriber("/ur5/joints", ur5Joints, self.joints_callback)
        rospy.Subscriber("/vrpn_client_node/POV/pose", PoseStamped, self.pov_callback)
        

    def joints_callback(self,msg):
        for idx, p in enumerate(msg.positions):
            self.ur5_joints[idx] = p

    def pov_callback(self,msg):
        self.pov_pose[0]= msg.pose.position.x
        self.pov_pose[1]= msg.pose.position.y
        self.pov_pose[2]= msg.pose.position.z
        self.pov_pose[3]= msg.pose.orientation.w
        self.pov_pose[4]= msg.pose.orientation.x
        self.pov_pose[5]= msg.pose.orientation.y
        self.pov_pose[6]= msg.pose.orientation.z


    def ros2unity_ur5(self):
        rel_tf = compose(self.pov_pose[0:3], quat2mat(self.pov_pose[3:]), [1,1,1])
        pov_msg = CamPose()
        # manually changing coordinate frame to match unity coordinate frame
        pov_msg.x  = -rel_tf[0,-1]
        pov_msg.y  =  rel_tf[1,-1] 
        pov_msg.z  =  rel_tf[2,-1]
        _q = mat2quat(rel_tf[0:3,0:3])
        pov_msg.x_ = -_q[1]
        pov_msg.y_ = _q[2]
        pov_msg.z_ = _q[3]
        pov_msg.w_ = -_q[0]
        self.pov_pub.publish(pov_msg)
        print(f"pov pose: {pov_msg}")
        

    def ros2unity_pov(self):
        msg = Ur5Joints()
        msg.joints = self.ur5_joints     
        self.cmd_pub.publish(msg)
        print(f"ur5 joints: {msg}")

if __name__ == '__main__':
        
   
   r2u = ROS2Unity()
   while not rospy.is_shutdown():
       try:
           print("running ros2unity node")
           r2u.ros2unity_ur5()
           r2u.ros2unity_pov()
           rospy.sleep(1/120)
       except Exception as e:
           print("main loop error ", e)




