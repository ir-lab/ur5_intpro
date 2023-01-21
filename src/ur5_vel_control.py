#!/usr/bin/env python3

import numpy as np
import kdl_parser_py.urdf
import PyKDL as KDL
import rospkg
import rospy
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl, ur5Tool


from ur5_intpro.msg import Ur5Joints as unityUr5
from transforms3d.affines import compose
from transforms3d.euler import mat2euler, euler2mat
from transforms3d.quaternions import quat2mat , mat2quat

import threading
import os 
import sys
import time

home_joints = [115.55, -105.80, 99.89, -84.24, -89.98, 25.42]
home_joints = [np.deg2rad(jh) for jh in home_joints]
waypoints = np.array([[-0.5, 0.5, 0.2],
                      [ 0.0, 0.5, 0.35],
                      [ 0.5, 0.5, 0.2],
                      [ 0.5, -0.1, 0.2],
                      [ 0.5, 0.5, 0.2],
                      [ 0.0, 0.5, 0.35]])

# waypoints = np.array([[-0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [ 0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [-0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [ 0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [-0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [ 0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [-0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [ 0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [-0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [ 0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [-0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2],
#                       [ 0.5, 0.5, 0.2],
#                       [ 0.0, 0.5, 0.2]
#                       ])


# ur5 joints callback for getting current joint position
def ur5joints_callback(msg):
    global ur5_joints, ur5_jvels
    ur5_joints = [urj for urj in msg.positions]
    ur5_jvels = [urv for urv in msg.velocities]

def unity_ur5joints_callback(msg):
    global unity_ur5_joints
    unity_ur5_joints = msg.joints

def unity_ur5joints_shadow_callback(msg):
    global unity_ur5_shadow_joints
    unity_ur5_shadow_joints = msg.joints
    
# one time setting up of kinematic chain using kdl
def setup_kdl():
    urdf_path = os.path.join(rospkg.RosPack().get_path("ur5_intpro"),"urdf","custom_ur5.urdf")
    kdl_tree  = kdl_parser_py.urdf.treeFromFile(urdf_path)[1]
    kdl_chain = kdl_tree.getChain("base_link","fake_end_effector_link") 
    global ur5_ik_solver, ur5_fk_solver
    ur5_ik_solver = KDL.ChainIkSolverPos_LMA(kdl_chain, 1e-8 ,1000, 1e-6)
    ur5_fk_solver = KDL.ChainFkSolverPos_recursive(kdl_chain)


# reset the pid ros paramters 
def set_pid_param(p = 1.0, i = 0.0, d = 0.0):
    rospy.set_param("/ur5/control_params/p_gain",p)
    rospy.set_param("/ur5/control_params/i_gain",i)
    rospy.set_param("/ur5/control_params/d_gain",d)
    
# run ros spin as separate thread to callback and publish msgs
def rosspin():
    rospy.spin()

# user defined homing positin for ur5
def go_home():
    set_pid_param( p = 2.1, 
                   i = 0.2,
                   d = 0.0
                   )
    rospy.sleep(1)
    traj_len = 200
    traj = [] * traj_len
    for c, g in zip(ur5_joints,home_joints):
        t_ = np.linspace(start=c, stop=g , num=traj_len)
        traj.append(t_)
    traj = np.array(traj).T        
    for t in traj:
        ur5_control_msg.values = t
        ur5_publisher.publish(ur5_control_msg)     
    print("Finished trajectory")
    print("home joints", home_joints)
    print("ur5 joints",ur5_joints)

# generate simple linspace trajectory
def get_trajectory(c_joints, g_joints, traj_len = 200):
    traj = [] * traj_len
    for c, g in zip(c_joints,g_joints):
        t_ = np.linspace(start=c, stop=g , num=traj_len)
        traj.append(t_)
    traj = np.array(traj).T        
    return traj

# move to multiple way points
def move2waypoints(traj_len,robot_type = "shadow"):
    print(f"Moveing {robot_type} robot" )
    if rospy.get_param("go_home",default=False):
        for _ in range(5):
            unity_ur5_msg = unityUr5()
            unity_ur5_msg.joints = home_joints
            
            if robot_type == "shadow":
                ur5_unity_shadow_publisher.publish(unity_ur5_msg)
            else:
                ur5_unity_publisher.publish(unity_ur5_msg)
                
            rospy.sleep(0.5)
            # rospy.set_param("go_home",False)
    
    for widx, w in enumerate(waypoints):
        ####--------- uncomment below for real ur5---------#### 
        g_joints = get_ik_sol(c_joints=ur5_joints,
                              xyz=w, 
                              rpy=[0,np.pi/2,0])
        
        traj = get_trajectory(c_joints=ur5_joints, 
                              g_joints= g_joints,
                              traj_len=traj_len)
        
        
        ####--------- uncomment below for unity ur5---------#### 
        global unity_ur5_joints, unity_ur5_shadow_joints
        
        if robot_type == "shadow":
            g_joints = get_ik_sol(c_joints = unity_ur5_shadow_joints,
                                xyz      = w, 
                                rpy      = [0,np.pi/2,0])
            
            traj = get_trajectory(c_joints = unity_ur5_shadow_joints, 
                                g_joints = g_joints,
                                traj_len = traj_len)
            
        else:
            g_joints = get_ik_sol(c_joints = unity_ur5_joints,
                                xyz      = w, 
                                rpy      = [0,np.pi/2,0])
            
            traj = get_trajectory(c_joints = unity_ur5_joints, 
                                g_joints = g_joints,
                                traj_len = traj_len)
            
        
        for t in traj:
            ur5_control_msg.values = t
            # testing for unity ur5
            unity_ur5_msg = unityUr5()
            unity_ur5_msg.joints = t
            
            if robot_type == "shadow":
                ur5_unity_shadow_publisher.publish(unity_ur5_msg)
            else:
                ur5_unity_publisher.publish(unity_ur5_msg)
            ur5_publisher.publish(ur5_control_msg) 
            rospy.sleep(1/(traj_len))
            
        # print(f"finished waypoint {widx} for robot_type: {robot_type}")
        # rospy.sleep(0.5)
        # counter = 0
        # while not reached_goal(ur5_joints,g_joints,abs_tol=0.01):
        #     counter += 1
        #     rospy.sleep(0.1)
        #     if counter > 50:
        #         break

# check if the current joints have reached to goal joints
# return True if reached goal joints else False  
# specifically userfull incase of position control 
def reached_goal(c_joints, g_joints, abs_tol = 1e-3, rel_tol = 1e-3):
    if len(c_joints) != len(g_joints):
        raise ValueError("missmatch between goal joints and current joints")
    if np.allclose(c_joints,g_joints, atol=abs_tol, rtol=rel_tol):
        return True
    else:
        return False 
    

# get inverse kinematic solution given goal xyz (meters), 
# rpy (radian) and current joint angles (radian)
def get_ik_sol(c_joints,xyz,rpy):
    kdl_c_joints = KDL.JntArray(6)
    for enum, j in enumerate(c_joints):
        kdl_c_joints[enum] = j
    kdl_xyz = KDL.Vector(*xyz)
    kdl_rpy = KDL.Rotation().EulerZYX(*rpy)
    kdl_g_joints = KDL.JntArray(6)
    global  ur5_ik_solver
    g_joints = ur5_ik_solver.CartToJnt(kdl_c_joints, KDL.Frame(kdl_rpy,kdl_xyz), kdl_g_joints)
    return [gj for gj in kdl_g_joints]

if __name__ == '__main__':
    rospy.init_node("ur5vel_control",anonymous=True)
    rospy.Subscriber("/ur5/joints",ur5Joints,ur5joints_callback)
    rospy.Subscriber("/unity_ur5/joints",unityUr5,unity_ur5joints_callback)
    rospy.Subscriber("/unity_ur5_shadow/joints",unityUr5,unity_ur5joints_shadow_callback)
    rp_thread  = threading.Thread(target=rosspin,args=())
    # rp_thread.start()
    
    set_pid_param( p = 3.0, 
                   i = 0.35,
                   d = 2.0
                   )
    rospy.sleep(1)
    setup_kdl()
    
    global ur5_publisher, ur5_unity_publisher, ur5_unity_shadow_publisher
    ur5_publisher = rospy.Publisher("/ur5/control",ur5Control,queue_size=1, latch=True)
    ur5_unity_publisher = rospy.Publisher("/ur5_goal/joints",unityUr5,queue_size=1, latch=True)
    ur5_unity_shadow_publisher = rospy.Publisher("/ur5_shadow/joints",unityUr5,queue_size=1, latch=True)
    global ur5_control_msg
   
    ur5_control_msg = ur5Control()
    ur5_control_msg.acceleration = 0
    ur5_control_msg.velocity = 0
    ur5_control_msg.blend = 0
    ur5_control_msg.gain = 0
    ur5_control_msg.command = "speedj"
    ur5_control_msg.jointcontrol = True
    # traj_len = 100
    traj_len = 500
    
    ur5_control_msg.time = 1/traj_len
    
    # go home
    rospy.set_param("go_home",True)

    import threading
    
    th1 = threading.Thread(target=move2waypoints,args=(traj_len,"shadow"))
    th2 = threading.Thread(target=move2waypoints,args=(traj_len,"real"))
    # th1.start()
    # th2.start()
    
    while not rospy.is_shutdown():
        time_delta = rospy.get_param("time_delta",default=0.1)
        
        try:
            # go_home()
           
            # move2waypoints(traj_len)
            if not th1.is_alive():
                print("starting new thread 1")
                th1 = threading.Thread(target=move2waypoints,args=(traj_len,"shadow"))
                th1.start()
            rospy.sleep(time_delta)
            
            if not th2.is_alive():
                print("starting new thread 2")
                th2 = threading.Thread(target=move2waypoints,args=(traj_len,"real"))
                th2.start()
                
            # move2waypoints(traj_len,robot_type="real")   
            # if not th1.is_alive():
            #     th1.join      
            # break
            # rospy.sleep(1/50)
            # rp_thread.join()
            # exit()
            # break
        except Exception as e:
            print(e)