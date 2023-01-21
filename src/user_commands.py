#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os
import rospkg


class USER_INT:
    def __init__(self) -> None:
        rospy.init_node("user_interface")
        rospy.set_param("start_user_exp",False)
        rospy.set_param("stop_user_exp",False)
        rospy.set_param("go_home",False)
        self.user_interface = Image()
        self.goal_ids = np.array([[6,5,4,3,2,1],
                                  [12,11,10,9,8,7],
                                  [18,17,16,15,14,13]],dtype=np.uint8)
        self.pattern = 255*np.ones((340,670,3),dtype=np.uint8)
        self.robot_goals = None
        
        
    def get_robot_goals(self,nums = 6):
        flat = np.reshape(self.goal_ids,(-1))
        return np.random.choice(flat,size=nums,replace=False)
        
    def generate_pattern(self):
        self.pattern = 255*np.ones((340,670,3),dtype=np.uint8)
        self.robot_goals = self.get_robot_goals()
        rsize = 100
        for i,goal in enumerate(self.goal_ids):
            for j,pose in enumerate(goal):
                x11 = 10 + j * 110
                y11 = 10 + i*110
                points = np.array([[x11,y11],[x11+rsize,y11], [x11+rsize, y11+rsize], [x11, y11+rsize]], dtype=np.int32)
                if pose in self.robot_goals:
                    self.pattern = cv2.fillConvexPoly(self.pattern, points, (0,0,255))
                else:
                    self.pattern = cv2.fillConvexPoly(self.pattern, points, (0,255,0))
                cv2.putText(self.pattern,str(pose),(x11+int(rsize/2), y11+int(rsize/2)),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),thickness=2)
                
    def run_experiment(self):
        try: 
            
            print("wating for input...")
            # cv_image = CvBridge().imgmsg_to_cv2(self.user_interface,desired_encoding="bgr8")
            
            cv2.imshow("Experiment",cv2.resize(self.pattern,(1000,500))) 
            cv_key = cv2.waitKey(1)
            if cv_key == 32: # space
                print("Stopping experiment")
                rospy.set_param("stop_user_exp",True)
                rospy.set_param("start_user_exp",False)
            
            elif cv_key == 13: # enter
                print("Starting experiment")
                rospy.set_param("start_user_exp",True)
                rospy.set_param("stop_user_exp",False)
            
            elif cv_key == ord('h'):
                rospy.set_param("go_home",True)
                
            if cv_key == ord('n'):
                # rospy.set_param("reinit_goals",True)
                rospy.set_param("new_mode",True)
                self.generate_pattern()
        
        except Exception as e:
            print(f"Run error: {e}")

if __name__ == '__main__':
    
    ui  =  USER_INT()
    ui.generate_pattern()
    while not rospy.is_shutdown():
        rospy.set_param("robot_goals",ui.robot_goals.tolist())
        ui.run_experiment()
        rospy.sleep(0.01)