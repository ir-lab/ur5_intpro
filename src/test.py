
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


global ur5_joints
ur5_joints = [0,0,0,0,0,0]

global pov_pose, obj_pose
pov_pose = [0,0,0,0,0,0,1]
obj_pose = [0,0,0,0,0,0,1]

def joints_callback(msg):
    for idx, p in enumerate(msg.positions):
        ur5_joints[idx] = p

def pov_callback(msg):
    pov_pose[0]= msg.pose.position.x
    pov_pose[1]= msg.pose.position.y
    pov_pose[2]= msg.pose.position.z
    pov_pose[3]= msg.pose.orientation.w
    pov_pose[4]= msg.pose.orientation.x
    pov_pose[5]= msg.pose.orientation.y
    pov_pose[6]= msg.pose.orientation.z

def obj_callback(msg):
    obj_pose[0]= msg.pose.position.x
    obj_pose[1]= msg.pose.position.y
    obj_pose[2]= msg.pose.position.z
    obj_pose[3]= msg.pose.orientation.w
    obj_pose[4]= msg.pose.orientation.x
    obj_pose[5]= msg.pose.orientation.y
    obj_pose[6]= msg.pose.orientation.z

def rosspin():
    rospy.spin()
       

def check_(pose: PoseStamped) -> PoseStamped:
    tf = compose([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
                 quat2mat([pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z]),
                 [1,1,1])
    
    tf2 = compose([0,0,0],np.eye(3),[1,1,1])
    tf  = np.matmul(tf,tf2)
    
    print(tf)
    new_pose = PoseStamped()
    new_pose.header.frame_id = "world"
    new_pose.pose.position.x = tf[0,-1]
    new_pose.pose.position.y = tf[1,-1]
    new_pose.pose.position.z = tf[2,-1]
    
    quat= mat2quat(tf[0:3,0:3])
    new_pose.pose.orientation.w = quat[0]
    new_pose.pose.orientation.x = quat[1]
    new_pose.pose.orientation.y = quat[2]
    new_pose.pose.orientation.z = quat[3]
    
    return new_pose
    
if __name__ == '__main__':


    
    # tf1  = compose([0,0,0],euler2mat(0,0,0),[1,1,1])    
    # tf2  = compose([0,0.1,0.5],euler2mat(0,0,0),[1,1,1])
    
    # tf3 = np.matmul(tf2,np.linalg.inv(tf1))
    # print(mat2euler(tf3[0:3,0:3]))
    
    # exit()
        
    rospy.init_node("test")
    # cmd_pub = rospy.Publisher("/arm_controller/command",JointTrajectory,queue_size=100)
    # group_pub = rospy.Publisher("/joint_group_position_controller/command",Float64MultiArray,queue_size=100)

    cmd_pub = rospy.Publisher("/real_ur5/joints",Ur5Joints,queue_size=1)
    pov_pub = rospy.Publisher("/pov_pose",CamPose,queue_size=1)
    relpose_pub = rospy.Publisher("/rel_pose",PoseStamped,queue_size=1)
    relpose2_pub = rospy.Publisher("/rel_pose2",PoseStamped,queue_size=1)
    rospy.Subscriber("/ur5/joints", ur5Joints, joints_callback)
    rospy.Subscriber("/vrpn_client_node/POV/pose", PoseStamped, pov_callback)
    rospy.Subscriber("/vrpn_client_node/Tennis_Ball/pose", PoseStamped, obj_callback)
    
    th = threading.Thread(target=rosspin,args=())
    th.start()
    while not rospy.is_shutdown():
        try:
            # msg = JointTrajectory()
            # msg.header.frame_id = "ur5"
            # msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            # positions = [0.0, -0., 0.0, 0.0, 0.0, 0.0]
            # velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # accelerations = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
            # effort = [0, 0, 0, 0, 0, 0]
            # # for i in range(len(positions)):
            # point = JointTrajectoryPoint()
            # point.positions = positions
            # point.velocities = velocities
            # point.accelerations = accelerations
            # point.effort = effort
            # point.time_from_start = rospy.Time.now()
            # msg.points.append(point)
            # print(msg)
            # cmd_pub.publish(msg)
            # print(obj_pose[3:])
            
            # negate both {x swapped with z} and vice versa
            # pos [x,y,z] -> pos[-z,y,-x]
            
            # quat rotate around y by -90 degrees
            # quat [-w,-x,y,z]
            
            # Changing the tf from opti to unity
            # new_obj_pose = [-obj_pose[2], obj_pose[1], -obj_pose[0]]
            # new_obj_quat = qmult(obj_pose[3:],euler2quat(0, -np.pi/2, 0))
            # new_obj_quat = [-new_obj_quat[0], -new_obj_quat[1], new_obj_quat[2], new_obj_quat[3]]            
            # tf_obj   = compose(new_obj_pose,quat2mat(new_obj_quat), [1,1,1])
            # # tf_obj   = compose(new_obj_pose,quat2mat([1,0,0,0]), [1,1,1])
            # # tf_obj = compose(obj_pose[0:3], quat2mat(obj_pose[3:]), [1,1,1])
            # # tf_obj = compose(obj_pose[0:3], quat2mat([1,0,0,0]), [1,1,1])
            # # tf_obj   = compose(new_obj_pose,quat2mat(new_obj_quat))
            
            
            # new_pov_pose = [-pov_pose[2], pov_pose[1], -pov_pose[0]]
            # # new_pov_quat = qmult(pov_pose[3:],euler2quat(0, -np.pi/2, 0))
            # new_pov_quat = qmult(pov_pose[3:],euler2quat(0, -np.pi/2, 0))
            # new_pov_quat = [-new_pov_quat[0], -new_pov_quat[1], new_pov_quat[2], new_pov_quat[3]]            
            # tf_pov = compose(new_pov_pose, quat2mat(new_pov_quat), [1,1,1])
            # # tf_pov = compose(new_pov_pose, quat2mat([1,0,0,0]), [1,1,1])
            # # tf_pov = compose(pov_pose[0:3], quat2mat(pov_pose[3:]), [1,1,1])
            # # tf_pov = compose(pov_pose[0:3], quat2mat([1,0,0,0]), [1,1,1])
            # rel_tf = np.matmul(tf_obj, np.linalg.inv(tf_pov))
            # # tf_adj = compose([0,0,0],euler2mat(0,np.pi/2,0),[1,1,1])
            # # rel_tf = np.matmul(tf_adj,rel_tf)
            
            # # print("Start")
            # # print(tf_obj)
            # # print(tf_pov)
            # # print(rel_tf)
            # tf_obj = compose(obj_pose[0:3], quat2mat([1,0,0,0]), [1,1,1])
            # tf_pov = compose(pov_pose[0:3], quat2mat([1,0,0,0]), [1,1,1])
            tf_obj = compose(obj_pose[0:3], quat2mat(obj_pose[3:]), [1,1,1])
            tf_pov = compose(pov_pose[0:3], quat2mat(pov_pose[3:]), [1,1,1])

            # rel_tf = np.matmul(tf_pov, np.linalg.inv(tf_obj))
            rel_tf = tf_pov
            
        
            
            pov_msg = CamPose()
            pov_msg.x  = -rel_tf[0,-1]
            pov_msg.y  = rel_tf[1,-1] 
            pov_msg.z  = rel_tf[2,-1]
            # pov_msg.z  = -rel_tf[2,-1]
            _q = mat2quat(rel_tf[0:3,0:3])
            # _q = euler2quat(0,np.pi,0)
            # _q = qmult(pov_pose[3:],euler2quat(-np.pi,0,-np.pi))
            # _e =list(quat2euler(_q))
            # _e[0] = -_e[0]
            # _e[1] = -_e[1]
            # _e[2] = -_e[2]
            # _q = euler2quat(*_e)
            # pov_msg.x_ = -_q[1]
            pov_msg.x_ = -_q[1]
            pov_msg.y_ = _q[2]
            pov_msg.z_ = _q[3]
            pov_msg.w_ = -_q[0]
            print(f"trans:  {rel_tf[0:3,-1]}")
            # print(f"euler:  {[np.rad2deg(e_) for e_ in  quat2euler(qmult(pov_pose[3:],euler2quat(np.pi,0,np.pi)))]}")
            print(f"euler:  {[np.rad2deg(e_) for e_ in  quat2euler(_q)]}\n")
            # testing
            # testing
            
            # pov_msg.x_ = 0.0#new_pov_quat[1]#pov_pose[4]#_q[1]
            # pov_msg.y_ = 0.0#new_pov_quat[2]#pov_pose[5]#_q[2]
            # pov_msg.z_ = 0.0#new_pov_quat[3]#pov_pose[6]#_q[3]
            # pov_msg.w_ = 1.0#new_pov_quat[0]#pov_pose[3]#_q[0]
            
            msg = Ur5Joints()
            msg.ur_joints_unity = ur5_joints

            relpose_msg = PoseStamped()
            relpose_msg.pose.position.x = pov_msg.x
            relpose_msg.pose.position.y = pov_msg.y
            relpose_msg.pose.position.z = pov_msg.z
            
            relpose_msg.pose.orientation.x = pov_msg.x_
            relpose_msg.pose.orientation.y = pov_msg.y_
            relpose_msg.pose.orientation.z = pov_msg.z_
            relpose_msg.pose.orientation.w = pov_msg.w_
            relpose_msg.header.frame_id = "world"
            
            
            # move_pose = check_(relpose_msg)
            
            relpose_pub.publish(relpose_msg)
            # relpose2_pub.publish(move_pose)
            cmd_pub.publish(msg)
            pov_pub.publish(pov_msg)
            
            # print(f"Publishing ur5 joints... {ur5_joints}")
            time.sleep(1/30)
            # rospy.sleep(0.01)
        except KeyboardInterrupt:
            print("shutdown")



