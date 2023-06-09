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
from ur5_intpro.msg import Ur5Joints as unityUr5
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
        self.r2fg_msg.force = 255
        self.r2fg_msg.speed = 255
        self.ur5_control_msg.command      = rospy.get_param("irl_robot_command",  default="movej") 
        self.ur5_control_msg.acceleration = rospy.get_param("irl_robot_accl",     default=np.pi/2)
        self.ur5_control_msg.velocity     = rospy.get_param("irl_robot_vel",      default=np.pi/2) 
        self.ur5_control_msg.time         = rospy.get_param("irl_robot_com_time", default=5)
        self.ur5_control_msg.jointcontrol = True
        self.ur5_control_timeout = 0
        self.unity_ur5_msg = unityUr5() 
        self.unity_msg_flag = False 
        rospy.Subscriber("/unity_ur5/joints",unityUr5,self.unity_ur5_callback)
        rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,buff_size=1)
        rospy.Subscriber("/ur5/joints",ur5Joints,self.ur5_joints_callback)
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        
        # kdl setup 
        self.urdf_path     = os.path.join(self.package_path,"urdf",self.general_params["kdl_urdf"])
        self.ur5_tf_chain  = self.init_kdl()
        self.ur5_ik_solver = KDL.ChainIkSolverPos_LMA(self.ur5_tf_chain, 1e-8 ,1000, 1e-6)
        self.ur5_fk_solver = KDL.ChainFkSolverPos_recursive(self.ur5_tf_chain)
        # self.ur5_ik_solver =  KDL.ChainIkSolverPos_NR(Chain=self.ur5_tf_chain)
        rospy.set_param("time_delta",1.0)
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
        self.unity_ur5_pub        = rospy.Publisher("/ur5_goal/joints",unityUr5,queue_size=1)
        self.unity_ur5_shadow_pub        = rospy.Publisher("/ur5_shadow/joints",unityUr5,queue_size=1)
        self.unity_robotiq_pub    = rospy.Publisher("/robotiq/joint",Float64,queue_size=1)
        # self.goals = self.general_params["goals"]
        print("Generating goals!!!!!")
        self.goals = self.generate_goals()
        self.robot_goals = None
        self.rosthread = self.rospy_thread()
        # self.socket_thread = threading.Thread(target=self.run_socket_packets,args=())
        # self.socket_thread.start()
        print("sleeping for a second.....")
        rospy.sleep(1)
        
    def decode_press(self,data):
        return struct.unpack('<f', data)[0]

   
    def unity_ur5_callback(self,msg):
        if msg is not None:
            self.unity_msg_flag = True
        else:
            self.unity_msg_flag = False
        self.unity_ur5_msg = msg
        
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
    
    def reached_goal(self,goal_joints,waypoint_id ="",real=False,rtol=1e-3,atol=1e-3,):
        if real:
            if waypoint_id == "workspace":
                
                if np.allclose(self.ur5_joints.positions,goal_joints, rtol=1e-1, atol=1e-1):
                    return True
                else:
                    return False
            else:
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
        timeout = 6
        counter = 0
        home_joints = [115.55, -105.80, 99.89, -84.24, -89.98, 25.42]
        home_joints = [np.deg2rad(jh) for jh in home_joints]
        for _ in range(100):
            # for idx, gj in enumerate(home_joints):
            #     self.sim_ur5_joint_publisher[idx].publish(gj)
            unity_home_joints = unityUr5()
            unity_home_joints.joints = home_joints
            self.unity_ur5_shadow_pub.publish(unity_home_joints)
            self.unity_ur5_pub.publish(unity_home_joints)
            rospy.sleep(0.01)
            
        self.publish_ur5_movej(goal_joints=home_joints)
        return
    
    
    def init_robot(self):
        self.go_home()
        self.r2fg_msg.position = 0
        self.r2fg_msg.force = 100
        self.r2fg_msg.speed = 255
        self.r2fg_control_publisher.publish(self.r2fg_msg)
        
    def get_time(self):
        return rospy.Time().now()
       
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
    def get_ik_sol(self, enable_rot=True, yaw_offset=0.0, real=False, unity=False):
        # get goal id (only for testing)
        kdl_init_joints = KDL.JntArray(6)
        if real:
            for i,jt in enumerate(self.ur5_joints.positions):
                kdl_init_joints[i] = jt
        elif unity:
            for i, jt in enumerate(self.unity_ur5_msg.joints):
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
                    self.r2fg_msg.position = int(v)
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
            # print(x,y,z)
        
        
        counter = 7
        for j in range(1,3):
            for rg in tmp_goals:
                if j ==2:
                    ref_goals.update({counter:[rg[0]+ (0.15*j), rg[1], rg[2]]})
                else:
                    ref_goals.update({counter:[rg[0]+ (0.15*j), rg[1], rg[2]]})
                counter += 1
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
    
    def real2sim(self,x,y,z):
        pass
    
    
    
    
    def run_thread(self):
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
                
                
    def testing_and_debug(self):
        goals = [[-0.4, 0.4, 0.2],
                 [-0.4, 0.4, 0.06],
                 [ 0.0, 0.4, 0.25],
                 [ 0.4, 0.4, 0.1],
                 [ 0.4, 0.4, 0.06],
                 [ 0.0, 0.4, 0.25]]
        if not self.unity_msg_flag:
            print("\nwaiting for unity joint msgs!!!!\n")
            return
        all_goals = self.generate_goals()
        self.robot_goals = rospy.get_param("robot_goals",default=list())
        self.robot_goals = self.robot_goals[:3]
        
        print("robot_goal ids: ",self.robot_goals)
        # if len(self.robot_goals) != 6:
        #     return
        home_joints = [115.55, -105.80, 99.89, -84.24, -89.98, 25.42]
        home_joints = [np.deg2rad(jh) for jh in home_joints]
        for _ in range(100):
            # for idx, gj in enumerate(home_joints):
            #     self.sim_ur5_joint_publisher[idx].publish(gj)
            unity_home_joints = unityUr5()
            unity_home_joints.joints = home_joints
            self.unity_ur5_shadow_pub.publish(unity_home_joints)
            self.unity_ur5_pub.publish(unity_home_joints)
            rospy.sleep(0.01)
            
        self.publish_ur5_movej(goal_joints=home_joints)
        
        while not rospy.is_shutdown():
            # for g_idx, g in enumerate(goals):
            for g in self.robot_goals:
                print("Moving to robot goal id: ",g)
                rg = all_goals.get(g)
                rospy.set_param("goal_id",g)
                waypoints = self.get_robot_waypoints(rg)
                
                for k, v in waypoints.items():
                    n_robot_goal = rospy.get_param("robot_goals",default=list())
                    if not np.allclose(self.robot_goals,n_robot_goal):
                        print("Got new goals!!!!!!!")
                        return 
                    
                    if k == "pick_up" or k == "pick_down":
                        self.ur5_control_msg.time = 0.8
                    else:
                        self.ur5_control_msg.time = 2.0
                        
                    
                    if k == "grasp" or k == "release":
                        # do robotic gripper action
                        self.r2fg_msg.position = v
                        self.r2fg_control_publisher.publish(self.r2fg_msg)
                        # pass
                    else:
                    
                        self.goal_x = v[0] 
                        self.goal_y = v[1] 
                        self.goal_z = v[2]
                        # sim_goal_joints = self.get_ik_sol()
                        sim_goal_joints  = self.get_ik_sol(unity=True,yaw_offset=np.pi/2)
                        real_goal_joints = self.get_ik_sol(unity=False,real=True,yaw_offset=np.pi/2)
                        
                        unity_goal_joints = unityUr5()
                        unity_goal_joints.joints = sim_goal_joints

                        # time delay
                        if k == "pick_up" or k == "pick_down" or k == "workspace" or k == "back":
                            delta = 0.0
                            self.ur5_control_msg.time = 1.2
                            
                        else:
                            delta = rospy.get_param("time_delta",default=1)

                        self.unity_ur5_shadow_pub.publish(sim_goal_joints)
                        rospy.sleep(delta)
                        self.ur5_control_msg.time = delta
                        if not rospy.get_param("halt"):
                            self.unity_ur5_pub.publish(unity_goal_joints)
                            self.publish_ur5_movej(goal_joints=real_goal_joints)
                        else:
                            print("halt..........................")
                        print(f"doing action in: {k} and delta: {delta}")
    
    
    def move_to_robot_goals(self):
        # self.robot_goals = self.robot_goals[:3]
        for g in self.robot_goals:
            print("Moving to robot goal id: ",g)
            rg = self.goals.get(g)
            rospy.set_param("goal_id",g)
            waypoints = self.get_robot_waypoints(rg)
            self.move_2_waypoints(waypoints=waypoints)
        return
    
    
    def iros_experiment(self):
        if rospy.get_param("go_home"):
                self.init_robot()
                rospy.set_param("go_home",False)
            
        if rospy.get_param("start_user_exp"):
            rospy.set_param("start_user_exp",False)                                    
            print("Starting the experiment")
            rospy.set_param("stop_saving",False)
            self.show_execution_time = False
            self.t1 = self.get_time()
            self.robot_goals = rospy.get_param("robot_goals",default=list())
            print("robot_goal ids: ",self.robot_goals)
            if len(self.robot_goals) != 6:
                return
            self.move_to_robot_goals()
            
            rospy.set_param("start_user_exp",False)
    
                
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
        return 
    
    def publish_ur5_movej(self,goal_joints,waypoint_id = ""):
        self.ur5_control_msg.values = goal_joints
        self.real_ur5_joint_publisher.publish(self.ur5_control_msg)
        while not rospy.is_shutdown():
            if not self.reached_goal(goal_joints=goal_joints,real=True,waypoint_id =waypoint_id):
                continue
            else:
                break   
        return
    
    def get_random_time(self, min=0.8, max=2.0):
        rsample = (max-min) * np.random.random_sample() + min
        return rsample
    
    def move_2_waypoints(self, waypoints):
        for k, v in waypoints.items():
            n_robot_goal = rospy.get_param("robot_goals",default=list())
            if not np.allclose(self.robot_goals,n_robot_goal):
                print("Got new goals!!!!!!!")
                return 
            
            if k == "pick_up" or k == "pick_down":
                self.ur5_control_msg.time = 0.8
            else:
                self.ur5_control_msg.time = 2.0
                
            
            if k == "grasp" or k == "release":
                # do robotic gripper action
                # self.r2fg_msg.position = v
                self.r2fg_msg.position = v if rospy.get_param("grasp_on") else 0
                self.r2fg_control_publisher.publish(self.r2fg_msg)
                # pass
            else:
            
                self.goal_x = v[0] 
                self.goal_y = v[1] 
                self.goal_z = v[2]
                # sim_goal_joints = self.get_ik_sol()
                
                real_goal_joints = self.get_ik_sol(unity=False,real=True,yaw_offset=np.pi/2)  
                sim_goal_joints  = self.get_ik_sol(unity=True,yaw_offset=np.pi/2)                
                unity_goal_joints = unityUr5()
                unity_goal_joints.joints = sim_goal_joints
                self.unity_ur5_shadow_pub.publish(sim_goal_joints)

                # time delay
                if k == "pick_up" or k == "pick_down" or k == "workspace" or k == "back":
                    delta = 0.0
                    # self.ur5_control_msg.time = 1.0
                    # self.ur5_control_msg.time = 1.0
                    
                else:
                    delta = rospy.get_param("time_delta",default=1)
                    
                # ur5 real control time
                if k == "pick_up" or k == "pick_down":
                    self.ur5_control_msg.time = self.get_random_time(min=0.4, max=1)
                
                elif k == " workspace" or k == "back":
                    self.ur5_control_msg.time = self.get_random_time(min=0.6, max=2)
                
                if k == "goal":  
                    rospy.set_param("start_static",True)
                    self.ur5_control_msg.time = self.get_random_time(min=0.9, max=2)
                    
                
                if rospy.get_param("mode_id",default="") == "noproj_mode":
                    delta = 0.0

                rospy.sleep(delta)
                
                
                if  k == "pick_up":
                    rospy.sleep(0.2)
                    rospy.set_param("start_static",False)
                
                if rospy.get_param("halt"):
                    while rospy.get_param("halt"):
                        rospy.sleep(0.1)  
                        print("halt........................")
                    
                self.unity_ur5_pub.publish(unity_goal_joints)
                self.publish_ur5_movej(goal_joints=real_goal_joints)
            
                # self.unity_ur5_pub.publish(unity_goal_joints)
                # self.publish_ur5_movej(goal_joints=real_goal_joints)
                
                print(f"doing action: {k}, ur5 control time: {self.ur5_control_msg.time} and delta: {delta}")
        return
    
    def set_pid_param(self,p = 1.0, i = 0.0, d = 0.0):
        rospy.set_param("/ur5/control_params/p_gain",p)
        rospy.set_param("/ur5/control_params/i_gain",i)
        rospy.set_param("/ur5/control_params/d_gain",d)        
        
    def __del__(self):
        pass
    
if __name__ == '__main__':
    rp = ROBOT_PRIMITIVES()
    rp.set_pid_param( p = 3.0, 
                      i = 0.35,
                      d = 2.0
                     )
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            print("running main loop")
            
            # uncomment below to run experiment of ICRA
            # rp.run_thread()
            
            # uncomment below to save goals (xyz) pose of objects aligned with projection
            # rp.save_robot_goals()
            
            # testing and debugging
            # rp.testing_and_debug()
            
                
            rp.iros_experiment()
                
            rospy.sleep(1/30)
        except Exception as e:
            print(f"Main loop error msg: {e}")