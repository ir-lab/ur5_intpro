#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from ur5_intpro.msg import Ur5Joints as UnityJoints
from irl_robots.msg import *
from transforms3d.euler import quat2euler, mat2euler
import time
import json
import numpy as np
from typing import Any
import subprocess
from robot_control import *
from threading import Thread


# class MotionTest(Robot_Control):
class MotionTest():
    
    def __init__(self) -> None:
        # super().__init__()      
        rospy.init_node("motion_planner")
        self.test_traj_dir = "/share/all_hd_data/101-240_dataset/"
        self.ur5Joints = ur5Joints()
        self.unityJoints = UnityJoints()
        self.ur5_control_msg = ur5Control()
        self.r2fg_msg        = gSimpleControl()
        self.r2fg_msg.force = 255
        self.r2fg_msg.speed = 255
        self.ur5_control_msg.command      = rospy.get_param("irl_robot_command",  default="movej") 
        self.ur5_control_msg.acceleration = rospy.get_param("irl_robot_accl",     default=np.pi/2)
        self.ur5_control_msg.velocity     = rospy.get_param("irl_robot_vel",      default=np.pi/2) 
        self.ur5_control_msg.time         = rospy.get_param("irl_robot_com_time", default=5)
        self.ur5_control_msg.jointcontrol = True

        rospy.Subscriber("/ur5/joints",ur5Joints,self.ur5_joints_callback)
        rospy.Subscriber("unity_ur5/joints",UnityJoints,self.unityur5_joints_callback)
        self.real_ur5_joint_publisher = rospy.Publisher("/ur5/control",ur5Control,queue_size=10)
        self.control_freq = 50.0
        
    def ur5_joints_callback(self,msg) -> None:
        self.ur5Joints = msg

    def unityur5_joints_callback(self,msg) -> None:
        self.unityJoints = msg
 
    def set_pid_param(self, p:float = 1.0, i:float = 0.0, d:float = 0.0, sleep:float = 0.5) -> None:
        rospy.set_param("/ur5/control_params/p_gain",float(p))
        rospy.set_param("/ur5/control_params/i_gain",float(i))
        rospy.set_param("/ur5/control_params/d_gain",float(d))
        rospy.sleep(sleep)
        
    def set_max_vel_accle(self, vel:float = np.pi/2.0, accel:float = np.pi/2.0, sleep:float = 0.5) -> None:
        rospy.set_param("/ur5/control_params/m_max_accel",accel)
        rospy.set_param("/ur5/control_params/m_max_velocity",vel)
        rospy.sleep(sleep)
    
    def movej_msg(self, goal_joints:list, vel:float = 0.0, accel:float = 0.0,  t:float = 5.0, blend = 0.0) -> None:
        if len(goal_joints) != 6:
            raise ValueError("Please provide all joint angles...")   
        self.ur5_control_msg.command = "movej"
        self.ur5_control_msg.time    = t
        self.ur5_control_msg.acceleration = vel
        self.ur5_control_msg.velocity = accel
        self.ur5_control_msg.blend = blend
        self.ur5_control_msg.jointcontrol = True
        self.ur5_control_msg.values = goal_joints
        
    
    def speedj_msg(self, goal_joints:list, t:float = 0.0, accl:float= 0.0) -> None:
        if len(goal_joints) != 6:
            raise ValueError("Please provide all joint angles...")
        self.ur5_control_msg.command = "speedj"
        self.ur5_control_msg.time = t
        self.ur5_control_msg.velocity = 0.0
        self.ur5_control_msg.acceleration = accl
        self.ur5_control_msg.jointcontrol = True
        self.ur5_control_msg.values = goal_joints

    def speedl_msg(self, d_xyz:list, d_rpy:list, accel:float = np.pi/4, t:float = 1.0) -> None:
        if len(d_xyz) != 3 and len(d_rpy) != 3:
            raise ValueError("Please check xyz and rpy values...")
        self.ur5_control_msg.command = "speedl"
        self.ur5_control_msg.time = t
        self.ur5_control_msg.velocity = 0.0
        self.ur5_control_msg.acceleration = accel
        self.ur5_control_msg.jointcontrol = True
        goal = [d for d in d_xyz] + [d for d in d_rpy]
        self.ur5_control_msg.values = goal
        
    def servoj_msg(self, goal_joints:list, t:float = 0.0, lookahead:float = 0.5, gain:int = 100) -> None:
        if len(goal_joints) != 6:
            raise ValueError("Please provide all joint angles...")
        self.ur5_control_msg.command = "servoj"
        self.ur5_control_msg.time = t
        self.ur5_control_msg.velocity = 0.0
        self.ur5_control_msg.acceleration = 0.0
        self.ur5_control_msg.jointcontrol = True
        self.ur5_control_msg.values = goal_joints
        self.ur5_control_msg.lookahead = lookahead
        self.ur5_control_msg.gain = gain
        
        
    def stopj_msg(self, accel = np.pi/4) -> None:
        self.ur5_control_msg.command = "stopj"
        self.ur5_control_msg.time = 0.0
        self.ur5_control_msg.velocity = 0.0
        self.ur5_control_msg.acceleration = accel
        self.ur5_control_msg.jointcontrol = True
        
        
    def get_random_traj(self) -> tuple:
        # only for custom trajectories....
        demos = os.listdir(self.test_traj_dir)
        # traj_id = "118" #np.random.choice(demos)
        traj_id = np.random.choice(demos)
        data_file = os.path.join(self.test_traj_dir,traj_id,"real_states.json")
        data = self.load_json(filename=data_file)
        trajectory = []
        tool_traj = []
        for i in range(len(data)):
            q_data   = data.get(f"real_{i}.png").get("q")
            e_xyz   = data.get(f"real_{i}.png").get("ee_pos")
            e_rotmat = np.array(data.get(f"real_{i}.png").get("ee_rot")).reshape(3,3)
            e_euler  = list(mat2euler(e_rotmat))
            # e_xyz = [e_xyz[1],e_xyz[0],e_xyz[2]]
            e_xyz[0] = -e_xyz[0]
            e_xyz[1] = -e_xyz[1] - 0.35774
            e_xyz[2] = e_xyz[2]
            e_euler[0] = -e_euler[0]
            e_euler[1] = -e_euler[1]
            # e_euler  = list([0,0,0])
            trajectory.append(q_data)
            tool_traj.append(e_xyz+e_euler)
        trajectory = np.array(trajectory)
        tool_trajectory = np.array(tool_traj)
        return trajectory,tool_trajectory
    
    def load_json(self, filename:str) -> dict:
        if not os.path.isfile(filename):
            raise ValueError(f"Folling json file does not exist: f{filename}")
        with open(filename, "r") as fh:
            data = json.load(fh)
        return data   
       
    def test_random_traj(self, random_traj:bool=False, use_speedj=False, use_servoj = True, use_movej=True,  use_speedl=False) -> None:
        trajectory, tool_trajectory  = self.get_random_traj()
        counter  = 0 
        prev_step  = 0
        while not rospy.is_shutdown():
            if counter == 0:
                self.movej_msg(goal_joints=trajectory[counter], t= 5.0)
                self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                rospy.sleep(5.0)
                prev_step = counter
                print(tool_trajectory[counter])
                # exit()
            else:
                if use_speedj:
                    self.speedj_msg(goal_joints=trajectory[counter], t=(1/self.control_freq))
                    self.real_ur5_joint_publisher.publish(self.ur5_control_msg) 
                
                elif use_servoj:
                    self.servoj_msg(goal_joints=trajectory[counter], t=0.05, lookahead=0.1, gain=500)
                    # self.servoj_msg(goal_joints=trajectory[counter], t=(1/self.control_freq), lookahead=0.04, gain=100)
                    self.real_ur5_joint_publisher.publish(self.ur5_control_msg) 
                    
                elif not use_speedj and use_speedl:
                    d_pose = tool_trajectory[counter]
                    print(d_pose)
                    
                    prev_step = counter
                    d_xyz = d_pose[0:3]
                    d_rpy = d_pose[3:]
                    self.speedl_msg(d_xyz,d_rpy)
                    self.real_ur5_joint_publisher.publish(self.ur5_control_msg)  
                    
                print(f"sending goal point: {counter+1}/{len(trajectory)}")  
            
            rospy.sleep(1/self.control_freq)
            if counter == len(trajectory)-1:
                self.stopj_msg(accel=2.0)
                self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                rospy.sleep(0.1)
                break
            else:
                counter += 1
    
    def start_rosbag(self,rosbag_path="/home/slocal/Downloads/2023-02-13-18-12-51.bag"):
        command = ['rosbag', 'play', '-r', '1.0', rosbag_path]
        subprocess.run(command)
    
    def test_rosbag_traj(self):
        counter  = 0 
        prev_step  = 0
        # th = Thread(target=self.start_rosbag,args=())
        # th.start()
        
        while not rospy.is_shutdown():
            goal_joints = list(self.unityJoints.joints)
            # goal_joints[-1] = 0.0
            if counter == -1:
                self.movej_msg(goal_joints=home_joints.get("rad"), t= 5.0)
                self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                rospy.sleep(5.0)
                prev_step = counter
                
            else:
                # self.speedj_msg(goal_joints=goal_joints, t=(1/self.control_freq))
                # self.real_ur5_joint_publisher.publish(self.ur5_control_msg) 
                # print(f"sending goal joints: {goal_joints}")
                self.servoj_msg(goal_joints=goal_joints, t=0.1, lookahead=0.2, gain=500)
                # self.servoj_msg(goal_joints=trajectory[counter], t=(1/self.control_freq), lookahead=0.04, gain=100)
                self.real_ur5_joint_publisher.publish(self.ur5_control_msg) 
                rospy.sleep(1/self.control_freq)
                
                # self.stopj_msg(accel=2.0)
                # self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                # rospy.sleep(1/self.control_freq)
            counter += 1
        
        
    
if __name__ == "__main__":
    
    mt = MotionTest()
    mt.set_pid_param(p = 4.5,
                     i = 0.0,
                     d = 50.0)
    mt.set_max_vel_accle(vel   = np.pi/2,
                         accel = np.pi/2)
    
    # for i in range(10):
    #     mt.test_random_traj()
    mt.test_rosbag_traj()
    # mt.test_random_traj(use_speedj=False, use_speedl=True)
    






























































# filename = "/share/all_hd_data/101-240_dataset/118/real_states.json"
# traj_data = None
# with open(filename,"r") as fh:
#     traj_data = json.load(fh)

# trajectory = []
# keys = []

# for i in range(len(traj_data)):
#     q_data = traj_data.get(f"real_{i}.png").get("q")
#     trajectory.append(q_data)
# # for k,v in traj_data.items():
# #     keys.append(k)
# #     for k1,v1 in v.items():
# #         if k1 == "q":
# #             trajectory.append(v1)

# trajectory = np.array(trajectory)
# print(trajectory.shape)

# keys.sort()
# print(len(traj_data))

# # exit()
# rospy.init_node("remaping")
# unity_ur5_pub = rospy.Publisher("/ur5_goal/joints",Ur5Joints,queue_size=1)
# ur5_pub  = rospy.Publisher("/ur5/control",ur5Control,queue_size=1)
# control_freq =  30
# real_ur5msg = ur5Control()
# # real_ur5msg.time = (1/control_freq)
# real_ur5msg.time = 0
# real_ur5msg.velocity = 1.57
# real_ur5msg.acceleration = 1.57
# # real_ur5msg.blend = 2.0
# real_ur5msg.command = "speedj"
# # real_ur5msg.command = "speedl"
# real_ur5msg.jointcontrol = True

# msg1 = []
# msg2 = []
# def unityUr5(msg):
#     global msg1, msg2
#     msg1 = msg
#     msg2 = list(msg.joints)
#     # msg1.append(msg)
#     # if len(msg1) > 10:
#     #     msg1.pop(-1)
#     # print(msg)
# rospy.Subscriber("unity_ur5/joints",Ur5Joints,unityUr5)
# # rospy.spin()

# def set_pid_param( p = 1.0, i = 0.0, d = 0.0):
#     rospy.set_param("/ur5/control_params/p_gain",p)
#     rospy.set_param("/ur5/control_params/i_gain",i)
#     rospy.set_param("/ur5/control_params/d_gain",d)
    
# def movej_exec(goal_joints):
#     real_ur5msg.command = "movej"
#     real_ur5msg.values = goal_joints
#     real_ur5msg.time = 5
#     ur5_pub.publish(real_ur5msg)
#     rospy.sleep(5)
# # set_pid_param( p = 6.0,
# #                i = 0.0,
# #                d = 4.8)

# set_pid_param( p = 3.2,
#                i = 0.00,
#                d = 2.5)
# rospy.set_param("/ur5/control_params/m_max_accel",0.75)

# rospy.sleep(1)
# counter = 0  
# while not rospy.is_shutdown():
#     # unity_ur5_pub.publish(msg1)
#     try:
#         if counter == 0:
#             movej_exec(trajectory[0])
#             rospy.sleep(1)
#         else:
#             goal_joint = trajectory[counter]
#             real_ur5msg.values = goal_joint
#             real_ur5msg.command = "speedj"
#             ur5_pub.publish(real_ur5msg)
#         print(f"moving to trajectory point {counter}")
#         # msg2[-1] = 0 
#         # real_ur5msg.values = msg2
#         # ur5_pub.publish(real_ur5msg)
#         rospy.sleep(1/control_freq)
#         if counter == len(traj_data)-1:
#             continue
#         else:
#             counter += 1
            
#     except KeyboardInterrupt:
#         # real_ur5msg.jointcontrol = True
#         # ur5_pub.publish(real_ur5msg)
#         # rospy.sleep(1/control_freq)
#         print("finished")
        