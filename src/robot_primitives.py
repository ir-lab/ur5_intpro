#! /usr/bin/env python3

import enum
from pickletools import read_stringnl_noescape_pair
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
import socket
import struct
import threading
import argparse
from six.moves import queue
import serial
from ds4_mapper import DS4_Mapper
from utils import ur5_intpro_utils
from tqdm import tqdm
import rosbag
import datetime

class ROBOT_PRIMITIVES:
    
    
    def __init__(self):
        rospy.init_node("ur5_kdl")
        
        self.package_path = rospkg.RosPack().get_path("ur5_intpro")
        self.rate = 5
        self.general_params = ur5_intpro_utils.load_yaml(os.path.join(self.package_path,"config","general_params.yaml"))
        self.execution_time = 0
        self.t1 = rospy.Time()
        self.t2 = rospy.Time()
        self.show_execution_time = False
        rospy.set_param("suction_gripper",False)
        rospy.set_param("go_home",True)
        # init ros msgs
        self.g_msg = Float64()

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
        self.ur5_fk_solver = KDL.ChainFkSolverPos_recursive(self.ur5_tf_chain)
        # self.ur5_ik_solver =  KDL.ChainIkSolverPos_NR(Chain=self.ur5_tf_chain)
        
        # ik prameters
        self.goal_x   = self.general_params["ik_goal"]["x"]
        self.goal_y   = self.general_params["ik_goal"]["y"]
        self.goal_z   = self.general_params["ik_goal"]["z"]
        self.yaw      = np.deg2rad(self.general_params["ik_goal"]["yaw"])
        self.pitch    = np.deg2rad(self.general_params["ik_goal"]["pitch"])
        self.roll     = np.deg2rad(self.general_params["ik_goal"]["roll"])
        
        self.sim_ur5_joint_publisher  = [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
        self.real_ur5_joint_publisher = rospy.Publisher("/ur5/control",ur5Control,queue_size=10)
        self.r2fg_control_publisher   = rospy.Publisher("/r2fg/simplecontrol",gSimpleControl,queue_size=10)
        
        # self.goals = self.general_params["goals"]
        print("Generating goals!!!!!")
        self.goals = self.generate_goals()
        self.rosthread = self.rospy_thread()
        # self.socket_thread = threading.Thread(target=self.run_socket_packets,args=())
        # self.socket_thread.start()
        print("sleeping for a second.....")
        rospy.sleep(1)
        
    def decode_press(self,data):
        return struct.unpack('<f', data)[0]

   
    
    def run_socket_packets(self):
        SERV = '192.168.1.124'
        HOST = '192.168.1.148'
        PORT = 15000

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sock.bind((HOST, PORT))
        count = 0
        motor = b'0'
        self.sock.sendto(b'OK', (SERV,PORT))
        print('triggered')
        p = rospy.Publisher("gripper_voltage",Float64,queue_size=1)
        electrodes = [rospy.Publisher(f"/electrode_{e}",Float64,queue_size=1) for e in range(8)]
        elctrodes_array = rospy.Publisher("/electrodes_voltage",Float64MultiArray,queue_size=1)
        # filename = f"/share/ikemoto_data/suction_gripper_data_{self.get_timestamp()}.bag"
        # bag  = rosbag.Bag(filename,'w')
            # rospy.set_param("record_bag",False)
            
        while not rospy.is_shutdown():
            # print("Running loop")
           
            msg, address = self.sock.recvfrom(8192)
            vols = self.decode_voltage(msg[:16])
            prss = self.decode_press(msg[-4:])
            self.sock.sendto(motor,(SERV,PORT))
            suction_griper = rospy.get_param("suction_gripper",default=False)
            
            pressure_msg = Float64()
            pressure_msg.data = prss
            # bag.write('gripper_voltage',pressure_msg)
            
            # print(len(vols))
            # print(vols)
            for idx,v in enumerate(vols):
                e_msg = Float64()
                e_msg.data = v
                electrodes[idx].publish(e_msg)
                # bag.write(f"electorde_{idx}",e_msg)
                rospy.sleep(0.001)
                
                
            # print(prss)
            msg = Float64()
            msg.data = float(prss)
            p.publish(msg)
            e_vols = Float64MultiArray()
            e_vols.data = vols
            elctrodes_array.publish(e_vols)
            # print("motor and count: ", motor, count)
            # if count == 100:
            #     if motor == b'0':
            #         motor = b'1'
            #     else:
            #         motor = b'0'
            #     count = 0
            # else:
            #     count += 1
                    
            
            if suction_griper == True:
                motor = b'1'
            else:
                motor = b'0'
            
            rospy.sleep(0.001)
        # bag.close()

    def get_timestamp(self):
        time_stamp = str(datetime.datetime.now()).replace(" ","_")
        time_stamp = time_stamp.replace("-","_")
        time_stamp = time_stamp.replace(":","_")
        time_stamp = time_stamp.replace(".","_")
        return time_stamp
    
    
    def decode_voltage(self,data, scale=10):
        decoded_data = []
        for i in range(len(data)):
            if i % 2 == 0:
                united_data = (data[i] & 0x0f) << 8 | data[i+1]
                voltage = scale * (united_data & 0x0fff) * (5 / 4096)
                decoded_data.append(voltage)

        return decoded_data

    def decode_imu(self,data):
        decoded_data = []
        f_list = data[0:8], data[8:16], data[16:24], data[24:32], data[32:40], data[40:48]
        for f in f_list:
            float_data = struct.unpack('>d', f)[0]
            decoded_data.append(float_data)

        return decoded_data
    
    def ur5_joints_callback(self,msg):
        self.ur5_joints = msg
             
    def joint_state_callback(self,msg):
        self.joint_state_msg = msg
    
    def joy_callback(self,msg):
        self.joy_msg = msg 
        
    def init_kdl(self,root_link="base_link",leaf_link="fake_end_effector_link"):
        kdl_tree = kdl_parser_py.urdf.treeFromFile(self.urdf_path)[1]
        return kdl_tree.getChain(root_link,leaf_link)
    
    
    def rospy_thread(self):
        rospy_thread = threading.Thread(target=rospy.spin,args=())
        rospy_thread.start()
        return rospy_thread
    
    def get_sim_joints(self):
        sim_joints = 6*[0]
        sim_joints[0] = self.joint_state_msg.position[3]
        sim_joints[1] = self.joint_state_msg.position[2]
        sim_joints[2] = self.joint_state_msg.position[0]
        sim_joints[3] = self.joint_state_msg.position[4]
        sim_joints[4] = self.joint_state_msg.position[5]
        sim_joints[5] = self.joint_state_msg.position[6]
        return sim_joints
    
    def get_time_delay(self):
        time_delay = rospy.get_param("time_delay",default=0.5) 
        return time_delay
    
    def reached_goal(self,goal_joints,real=False,rtol=1e-3,atol=1e-3):
        if real:
            if np.allclose(self.ur5_joints.positions,goal_joints, rtol=rtol, atol=atol):
                return True
            else:
                return False
        else:
            
            if np.allclose(self.get_sim_joints(),goal_joints, rtol=rtol, atol=atol):
                return True
            else:
                return False
        
    
    def go_home(self):
        print("Going Home!!!")
        home_joints = [np.deg2rad(jnt) for jnt in self.general_params["home_joints"]]
        timeout = 6
        counter = 0
        while not rospy.is_shutdown():
            if not (self.reached_goal(home_joints,real=True) or self.reached_goal(home_joints)):
                self.ur5_control_msg.values = home_joints
                self.ur5_control_msg.time = 3
                for i, gj in enumerate(home_joints):
                    self.sim_ur5_joint_publisher[i].publish((gj - np.pi/2) if i ==0 else gj)
                self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                rospy.sleep(3)
                if counter >= timeout:
                    break
                counter += 3
            else:
                break
        euler,_,_ = self.get_fk_frame(joints=self.get_sim_joints())
        print("Home euler",euler)
        return
    
    def init_robot(self):
        self.go_home()
        self.r2fg_msg.position = 0
        self.r2fg_msg.force = 100
        self.r2fg_msg.speed = 255
        self.r2fg_control_publisher.publish(self.r2fg_msg)
        
    def get_time(self):
        return rospy.Time().now()
    
    def run_thread(self):
        print("Running main Thread!!!")
        while not rospy.is_shutdown():
            try:
                print("runing main thread")
                
                if rospy.get_param("go_home"):
                    self.init_robot()
                    rospy.set_param("go_home",False)
                
                if rospy.get_param("start_user_exp"):
                    rospy.set_param("start_user_exp",False)                                    
                    print("Starting the experiment")
                    rospy.set_param("stop_saving",False)
                    self.show_execution_time = False
                    self.t1 = self.get_time()
                    self.ur5_publisher()
                    # rospy.set_param("start_user_exp",False)
                    
                if rospy.get_param("stop_user_exp"):
                    rospy.set_param("stop_user_exp",False)
                    print("Stopping the experiment")
                    self.show_execution_time = True
                    self.t2  = rospy.Time().now()
                    self.execution_time = (self.t2-self.t1) * 1e-9
                
                if self.show_execution_time:
                    rospy.set_param("stop_saving",True)
                    print(f"Execution time: {self.execution_time} secs")
                    self.show_execution_time = False
                
                print(rospy.get_param("stop_saving"))
                # else:
                #     rospy.set_param("stop_saving",False)
                    
                    
                rospy.sleep(1)
            except Exception as e:
                print(e)
    
    # obtain the fk: euler, pose, and 4x4 mat
    def get_fk_frame(self,joints, segmentNr=-1):
        joints_ = KDL.JntArray(6)
        frame   = KDL.Frame()
        for i in range(6):
            joints_[i] = joints[i]
        self.ur5_fk_solver.JntToCart(joints_,frame, segmentNr= segmentNr)
        rot_mat = []
        for i in range(3):
            for j in range(3):
                rot_mat.append(frame.M[i,j])
        rot_mat = np.array(rot_mat).reshape(3,3)
        euler  = mat2euler(rot_mat)
        pose = []
        for i in range(3):
            pose.append(frame.p[i])
        # print("Euler RPY: {}".format(euler))
        print("Trans XYZ: {}".format(pose))
        tf_mat = compose(pose,rot_mat,[1,1,1])
        return euler, pose, tf_mat

    # called given change in global goal position
    def get_ik_sol(self, enable_rot=True, yaw_offset=0, real=False):
        # get goal id (only for testing)
        kdl_init_joints = KDL.JntArray(6)
        if real:
            for i,jt in enumerate(self.ur5_joints.positions):
                kdl_init_joints[i] = jt
        else:            
            kdl_init_joints[0] = self.joint_state_msg.position[3] 
            kdl_init_joints[1] = self.joint_state_msg.position[2] 
            kdl_init_joints[2] = self.joint_state_msg.position[0]
            kdl_init_joints[3] = self.joint_state_msg.position[4]
            kdl_init_joints[4] = self.joint_state_msg.position[5]
            kdl_init_joints[5] = self.joint_state_msg.position[6] 
            # self.roll = 0.5764390002892801
            # self.pitch = np.pi/2
            # self.yaw   = 0.5764390002892801
        if yaw_offset:
            print("Using yaw offset")
            tf = compose([0,0,0],euler2mat(0,0,yaw_offset),[1,1,1])
            if real:    
                self.z_offset = self.goal_z - 0.09
            else:
                self.z_offset = self.goal_z
            goal_tf = compose([self.goal_x,self.goal_y,self.z_offset],euler2mat(self.yaw,self.pitch,self.roll),[1,1,1])
            goal_tf = np.matmul(tf,goal_tf)
            # print(goal_tf[0:3,-1])
            _xyz = KDL.Vector(goal_tf[0,-1],goal_tf[1,-1],goal_tf[2,-1])
            rot = mat2euler(goal_tf[0:3,0:3])
            _rpy = KDL.Rotation().EulerZYX(rot[2],rot[1],rot[0]) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
        else:
            if real:    
                self.z_offset = self.goal_z - 0.09
            else:
                self.z_offset = self.goal_z
            
            _xyz               = KDL.Vector(self.goal_x, self.goal_y, self.z_offset)     
            _rpy               = KDL.Rotation().EulerZYX(self.roll, self.pitch, self.yaw) if enable_rot else KDL.Rotation().EulerZYX(0, 0, 0)
        kdl_goal_frame     = KDL.Frame(_rpy,_xyz)
        kdl_goal_joints    = KDL.JntArray(6)
        self.ur5_ik_solver.CartToJnt(kdl_init_joints, kdl_goal_frame, kdl_goal_joints)
        kdl_goal_joints = [float(j) for j in kdl_goal_joints]
        return kdl_goal_joints
    
    
    # send ik solutions to ur5 robot
    def ur5_publisher(self):        
        robot_goals = rospy.get_param("robot_goals",default=[])

        if not robot_goals :
            print("waiting for robot goals over parameter server")
            return
        
        rf_goals = self.generate_goals()
        # all_goals = np.array(self.goals)
        # rf_goals = np.take(all_goals,robot_goals,axis=0)
        
        # for idx,g in enumerate(goals):
        for i in range(len(robot_goals)):
            g = rf_goals[robot_goals[i]]
            print(f"going to goal id: {robot_goals[i]}...\n")
            
            waypoints = self.get_robot_waypoints(g)
            for k,v in waypoints.items():
                print(f"Going to {k}")                    
                if k == "grasp" or k == "release":
                    # self.r2fg_msg.position = 0
                    self.r2fg_msg.position = v
                    self.r2fg_msg.force = 100
                    self.r2fg_msg.speed = 255
                    self.r2fg_control_publisher.publish(self.r2fg_msg)
                    rospy.sleep(0.5)
                else:
                    self.goal_x = v[0]
                    self.goal_y = v[1]
                    self.goal_z = v[2]

                    sim_goal_joints = self.get_ik_sol(yaw_offset=0)
                    real_goal_joints = self.get_ik_sol(yaw_offset=np.pi/2, real=True)
                    self.ur5_control_msg.values = real_goal_joints
                    
                    current_mode = rospy.get_param("current_mode")
                    if k == "pick_up" or k == "pick_down":
                        self.ur5_control_msg.time = 0.6
                        time_delay = 0 
                    elif  k == "workspace" or k == "back":
                        self.ur5_control_msg.time = 1.5
                        # if current_mode == "noproj_mode":
                        #     time_delay = 0
                        # else:
                        #     time_delay = 0 if k == "workspace" else self.get_time_delay()
                        time_delay = 0 if k == "workspace" else self.get_time_delay()
                    else:
                        self.ur5_control_msg.time = 1.5
                        # if current_mode == "noproj_mode":
                        #     time_delay = 0
                        # else:
                        #     time_delay =  self.get_time_delay()
                        time_delay =  self.get_time_delay()
                            
                            
                    for i, gj in enumerate(sim_goal_joints):
                        self.sim_ur5_joint_publisher[i].publish(gj)

                    rospy.sleep(time_delay)
                    self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                    while not rospy.is_shutdown():
                        if not self.reached_goal(goal_joints=real_goal_joints,real=True):
                            continue
                        else:
                            break     
        return 
    
    
    # Generic pick and place way points 
    # based on goal object's position
    def get_robot_waypoints(self,goal):
        waypoints = {}
        waypoints.update({"goal":goal})
        pick_down = [goal[0],goal[1],0.09]
        waypoints.update({"pick_down":pick_down})
        waypoints.update({"grasp":int(155)})
        pick_up = goal
        waypoints.update({"pick_up":pick_up})        
        back = [-0.10, 0.45*(1 if goal[1] >= 0 else -1),goal[2]]
        # back = [-0.10, 0.25*(1 if np.random.rand() > 0.5 else -1),goal[-1]]
        waypoints.update ({"back":back})
        waypoints.update({"release":0})
        workspace = [0.25, 0.45*(1 if goal[1] >= 0 else -1),goal[2]]
        waypoints.update({"workspace":workspace})
        
        return waypoints
        
    
    def generate_goals(self):
        # first six goals
        ref_goals = {}
        tmp_goals = []
        for i in range(6):
            tmp = np.load(os.path.join(self.package_path,"robot_goals",f"{i+1}.npy"))
            if i == 3 or i == 5:
                x = tmp[1,-1]-0.02
            else:
                x = tmp[1,-1]
            y = -tmp[0,-1]
            z = 0.28
            ref_goals.update({(i+1):[x,y,z]})
            tmp_goals.append([x,y,z])
            print(x,y,z)
        
        
        counter = 7
        for j in range(1,3):
            for rg in tmp_goals:
                if j ==2:
                    ref_goals.update({counter:[rg[0]+ (0.15*j), rg[1], rg[2]]})
                else:
                    ref_goals.update({counter:[rg[0]+ (0.15*j), rg[1], rg[2]]})
                counter += 1
        
        # all_goals = ref_goals + all_goals

        # print(all_goals)
        # exit()
                
        # ref_goal = np.load(os.path.join(self.package_path,"robot_goals",f"{1}.npy"))
        # ref_goal = compose([0.30,0.525,0.25], euler2mat(0,0,0),[1,1,1])
        # current_goal = ref_goal
        # goals = []
        # goals.append(ref_goal[0:3,-1])
        
        # c_y_offset= 0.0
        # y_change = 0
        # x_change = 0
        # for i in tqdm(range(3)):
        #     y_change = 0
        #     c_y_offset= 0.0
        #     for j in range(6):
        #         change_tf = compose([x_change,y_change,0],euler2mat(0,0,0),[1,1,1])
        #         new_goal = np.matmul(ref_goal,change_tf)
        #         goals.append(new_goal[0:3,-1])
        #         # if j >= 4 :
        #         #     c_y_offset += 0.00
                    
        #         if j >= 3:
        #             c_y_offset -= 0.0
        #         y_change -= (0.20 + c_y_offset) 
                
        #     x_change += 0.18
        # return goals
        return ref_goals
    
    def save_robot_goals(self):
        while not rospy.is_shutdown():
            print("waiting to save goals!!!")
            if rospy.get_param("save_robot_goal",default=False):
                print("saving goal......")
                goal_id = rospy.get_param("get_goal_id",default=0)
                print(f"saving goal {goal_id}")
                _,pose,tf_mat = self.get_fk_frame(joints=self.ur5_joints.positions)
                print(f"pose: {pose}")
                goal_file = os.path.join(self.package_path,"robot_goals",f"{goal_id}.npy")
                np.save(goal_file,tf_mat)
                rospy.set_param("save_robot_goal",False)
                rospy.sleep(2)
            rospy.sleep(0.1)
    
    
    def testing_and_debug(self):
        goals = [[0.5,-0.4, 0.2],
                 [0.5,-0.4, 0.06],
                 [0.5, 0.0, 0.25],
                 [0.5, 0.5, 0.1],
                 [0.5, 0.5, 0.06],
                 [0.5, 0.0, 0.25]]
        tmp =   0.04    
        # filename = f"suction_gripper_data_{self.get_timestamp()}"
        # bag  = rosbag.Bag(filename,'w')
        
        
        
        # go home if true
        if rospy.get_param("go_home"):
            self.go_home()
        counter_ = 0 
        gc_pub = rospy.Publisher("gripper_command",Float64,queue_size=1)
        while not rospy.is_shutdown():
               
            for g_idx, g in enumerate(goals):
                
                # if rospy.get_param("go_home"):
                #     self.init_robot()
                #     rospy.set_param("go_home",False)
                # print("gggggggggg")
                if g_idx == 0 or g_idx == 1:
                    self.goal_y = g[1] - tmp
                else:
                    self.goal_y = g[1] 
                
                    
                self.goal_x = g[0] 
                self.goal_z = g[2]
                 
                sim_goal_joints = self.get_ik_sol()
                for idx, gj in enumerate(sim_goal_joints):
                    self.sim_ur5_joint_publisher[idx].publish(gj)
                # print("goal_joints",goal_joints)
                
                real_curr_joints = [j for j in self.ur5_joints.positions]
                real_goal_joints = self.get_ik_sol(yaw_offset=np.pi/2, real=True)
                t_len = 50
                trajectory = [] * t_len
                for c, g in zip(real_curr_joints,real_goal_joints):
                    # print("current_j: ", c)
                    # print("goal_j: ", g)
                    traj = np.linspace(start=c, stop=g, num=t_len)
                    # print("trajectory: ",traj)
                    trajectory.append(traj)
                trajectory = np.array(trajectory).T
                print(trajectory.shape)
                self.ur5_control_msg.command = "speedj"
                # self.ur5_control_msg.acceleration = 0.5
                self.ur5_control_msg.velocity = 0
                self.ur5_control_msg.gain = 0
                self.ur5_control_msg.time = 1/t_len
                for t in trajectory:
                    # print(t)
                    self.ur5_control_msg.values = t
                    self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                    # rospy.sleep(0.1)
                    rospy.sleep(1/t_len)
                # for position control 
                # self.ur5_control_msg.values = real_goal_joints
                # self.ur5_control_msg.time = 5
                # self.ur5_control_msg.command = "movel"
                # self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
                
                # while not rospy.is_shutdown():
                #         if not self.reached_goal(goal_joints=real_goal_joints,real=True,atol=0.01, rtol=0.01):
                #             continue
                #         else:
                #             break     
                print("Suction gripper on !!!!!",g_idx)
                
                if g_idx  == 0:
                    rospy.set_param("suction_gripper",True)
                    rospy.sleep(0.5)
                    self.g_msg.data = 1
                elif g_idx == 4:
                    rospy.set_param("suction_gripper",False)
                    self.g_msg.data = 0
                    
                gc_pub.publish(self.g_msg)
                rospy.sleep(0.5)
            tmp *= 2
            counter_ += 1
            if counter_ == 3:
                break
            
            
    def __del__(self):
        pass
    
if __name__ == '__main__':
    
    try:
        rp = ROBOT_PRIMITIVES()
        # uncomment below to run experiment of ICRA
        # rp.run_thread()
        
        # uncomment below to save goals (xyz) pose of objects aligned with projection
        # rp.save_robot_goals()
        
        # testing and debugging
        rp.testing_and_debug()
    except Exception as e:
        print(e)