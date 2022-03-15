#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os
import rospkg







def imageCallback(msg):
    cv_image = CvBridge().imgmsg_to_cv2(msg,desired_encoding="bgr8")
    cv2.imshow("Pattern",cv_image)
    key = cv2.waitKey(1)
    if key == ord("s"):
        print("Starting experiment!")
        rospy.set_param("stop_user_exp",False)
        rospy.set_param("start_user_exp",True)
    
    if key == 32:
        rospy.set_param("start_user_exp",False)
        print("Stopping the experiment")
        rospy.set_param("stop_user_exp",True)

if __name__ == '__main__':
    
    rospy.init_node("user_commands")
    
    rospy.Subscriber("pattern_image",Image,imageCallback)
    rospy.spin()