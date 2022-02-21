#! /usr/bin/env python


from re import S
import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from rospy.client import get_param
from std_msgs.msg import String, Float64, Float64MultiArray
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl
from sensor_msgs.msg import JointState, Joy
from transforms3d.affines import compose
from transforms3d.euler import mat2euler, euler2mat
from transforms3d.quaternions import quat2mat , mat2quat
import threading
import os
import sys




class PNP:

    def __init__(self):

        rospy.init_node("v2_pick_and_place")
        
        self.ur5_intpro_dir = rospkg.RosPack().get_path("ur5_intpro")

        # for simulation ur5
        self.joint_state_msg = JointState()

        # for real ur5
        self.ur5_control_msg = ur5Control()
        self.ur5_joints = ur5Joints()
        self.r2fg_msg = gSimpleControl()

        # cartesian goal 
        self.trans = np.array([0.20, 0.40, 0.35],dtype=np.float)
        self.euler = np.array([-np.pi/2, 0, -np.pi],dtype=np.float) # for vertical position
        # self.euler = np.array([-np.pi/2, 0, np.pi/2],dtype=np.float) # for horizontal position

        # kdl setup 
        self.urdf_path = os.path.join(self.ur5_intpro_dir,"urdf/kdl_custom_ur5.urdf")
        self.ur5_tf_chain = self.init_kdl()
        self.ur5_ik_solver =  KDL.ChainIkSolverPos_LMA(self.ur5_tf_chain, 1e-8 ,1000, 1e-6)

        

        rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,buff_size=1)
        rospy.Subscriber("/ur5/joints",ur5Joints,self.ur5_joints_callback)

        self.ur5_joint_publisher = [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
        self.real_ur5_joint_publisher = rospy.Publisher("/ur5/control",ur5Control,queue_size=10)
        self.r2fg_control_publisher = rospy.Publisher("/r2fg/simplecontrol",gSimpleControl,queue_size=10)
           
        self.goals = {0:[0.30,  0.58, 0.08],
                      1:[0.30,  0.38, 0.08],
                      2:[0.30,  0.18, 0.08],
                      3:[0.30, -0.02, 0.08],     
                      4:[0.30, -0.22, 0.08],     
                      5:[0.30, -0.40, 0.08],
                      6: [0.46,  0.60, 0.08],
                      7: [0.46,  0.40, 0.08],
                      8: [0.46,  0.20, 0.08],     
                      9: [0.46, -0.00, 0.08],  
                      10:[0.46, -0.20, 0.08],
                      11:[0.46, -0.40, 0.08],
                      12:[0.64,  0.60, 0.08],
                      13:[0.64,  0.40, 0.08],     
                      14:[0.64,  0.20, 0.08],  
                      15:[0.64, -0.00, 0.08],  
                      16:[0.64, -0.20, 0.08],  
                      17:[0.64, -0.40, 0.08],  
                      }

    def init_kdl(self,root_link="base_link",leaf_link="tool0"):
        kdl_tree = kdl_parser_py.urdf.treeFromFile(self.urdf_path)[1]
        return kdl_tree.getChain(root_link,leaf_link)
    

    def ur5_joints_callback(self,msg):
        self.ur5_joints = msg
    
    def joint_state_callback(self,msg):
        self.joint_state_msg = msg

    
    def get_ik(self):

        init_joints = KDL.JntArray(6)
        init_joints[0] = self.joint_state_msg.position[3] 
        init_joints[1] = self.joint_state_msg.position[2] 
        init_joints[2] = self.joint_state_msg.position[0]
        init_joints[3] = self.joint_state_msg.position[4]
        init_joints[4] = self.joint_state_msg.position[5]
        init_joints[5] = self.joint_state_msg.position[6] 

        # compensate base offset 
        # goal_tf = compose(self.trans,euler2mat(self.euler[0,0],self.euler[0,1],self.euler[0,2]),[1,1,1])
        # goal_tf = np.matmul(compose([0,0,0],euler2mat(0,0,np/4),[1,1,1]))
        # self.trans = goal_tf[0:3,-1]
        # self.euler = mat2euler(self.goal_tf[0:3,0:3])
        goal_xyz = KDL.Vector(self.trans[0],
                              self.trans[1],
                              self.trans[2]+0.15) # z offset for end effector
    
        goal_ypr = KDL.Rotation().EulerZYX(self.euler[0],
                                           self.euler[1],
                                           self.euler[2])
        
        goal_frame = KDL.Frame(goal_ypr,goal_xyz)
        goal_joints = KDL.JntArray(6)
        self.ur5_ik_solver.CartToJnt(init_joints,goal_frame,goal_joints)

        return goal_joints


    def pnp_primitve(self):

        goal_id = rospy.get_param("goal_id",default=1000)
        if goal_id == 1000:
            return
        try:
            self.trans = self.goals[goal_id]
            
            # Go over the object
            self.trans[-1] = 0.25
            print("Goal trans value: {}".format(self.trans))
            goal_joints = self.get_ik()
            # print("goal joints: {}".format(goal_joints))
            # publish first on simulated ur5
            for i,jnt in enumerate(goal_joints):
                self.ur5_joint_publisher[i].publish(jnt)
            
            rospy.sleep(1)

            # Go to the object
            self.trans[-1] = 0.08
            print("Goal trans value: {}".format(self.trans))
            goal_joints = self.get_ik()
            # print("goal joints: {}".format(goal_joints))
            # publish first on simulated ur5
            for i,jnt in enumerate(goal_joints):
                self.ur5_joint_publisher[i].publish(jnt)


            # Grasp object
            

            rospy.sleep(1)

            # Go over the object
            self.trans[-1] = 0.25
            print("Goal trans value: {}".format(self.trans))
            goal_joints = self.get_ik()
            # print("goal joints: {}".format(goal_joints))
            # publish first on simulated ur5
            for i,jnt in enumerate(goal_joints):
                self.ur5_joint_publisher[i].publish(jnt)


            rospy.sleep(1)
            # Go to the final way point
            self.trans[0] = 0.2
            if self.trans[1]>0:
                self.trans[1] = 0.4
            else:
                self.trans[1] = -0.4
            self.trans[2] = 0.25
            goal_joints = self.get_ik()
            # print("goal joints: {}".format(goal_joints))
            # publish first on simulated ur5
            for i,jnt in enumerate(goal_joints):
                self.ur5_joint_publisher[i].publish(jnt)
        

        except KeyError:
            print("Goal not found for goal id: {}".format(goal_id))






if  __name__ == '__main__':

    pnp = PNP()
    rate = rospy.Rate(10)

    rospy.sleep(1)
    while not rospy.is_shutdown():
        pnp.pnp_primitve()
        rate.sleep()

