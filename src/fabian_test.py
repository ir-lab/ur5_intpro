#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from ur5_intpro.msg import Ur5Joints
from irl_robots.msg import ur5Control
import time


rospy.init_node("remaping")
unity_ur5_pub = rospy.Publisher("/ur5_goal/joints",Ur5Joints,queue_size=1)
ur5_pub  = rospy.Publisher("/ur5/control",ur5Control,queue_size=10)
control_freq =  100
real_ur5msg = ur5Control()
real_ur5msg.time = (1/control_freq)
real_ur5msg.command = "speedj"
real_ur5msg.jointcontrol = True

msg1 = []
msg2 = []
def unityUr5(msg):
    global msg1, msg2
    msg1 = msg
    msg2 = list(msg.joints)
    # msg1.append(msg)
    # if len(msg1) > 1000:
        # msg1 = []
    # print(msg)
rospy.Subscriber("unity_ur5/joints",Ur5Joints,unityUr5)
# rospy.spin()

def set_pid_param( p = 1.0, i = 0.1, d = 0.0):
    rospy.set_param("/ur5/control_params/p_gain",p)
    rospy.set_param("/ur5/control_params/i_gain",i)
    rospy.set_param("/ur5/control_params/d_gain",d)
    
    
set_pid_param( p = 6.0,
               i = 0.0,
               d = 4.8)
rospy.sleep(1)
while not rospy.is_shutdown():
    # unity_ur5_pub.publish(msg1)
    msg2[-1] = 0 
    real_ur5msg.values = msg2
    ur5_pub.publish(real_ur5msg)
    rospy.sleep(1/control_freq)
    
