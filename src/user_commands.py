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
        rospy.set_param("grasp_on",True)
        self.grasp_flag = True
        self.user_interface = Image()
        self.goal_ids = np.array([[6,5,4,3,2,1],
                                  [12,11,10,9,8,7],
                                  [18,17,16,15,14,13]],dtype=np.uint8)
        
        
        self.goal_ids_new = np.array([[6,12,18,17,16,15,14,13],
                                      [5,4,3,2,1,11,10,9,8,7]],dtype=object)
        self.robot_goals = self.get_robot_goals()
        
        self.pattern = 255*np.ones((340,670,3),dtype=np.uint8)
        self.c_goal = -100 #pas_goal
        self.p_goal = -100 #past goal
        
    def show_current_goal(self):
        goal = rospy.get_param("goal_id",default=100)
        mat_idx = np.where(self.goal_ids == self.c_goal)
        points, textloc = self.get_points(row=mat_idx[0][0],col=mat_idx[1][0])
        self.pattern = cv2.fillConvexPoly(self.pattern, points, (0,155,255))
        cv2.putText(self.pattern,str(goal),textloc,cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),thickness=2)        
        print(f"current goal: {goal}")
        
    def get_robot_goals(self,nums = 6, split =3):
        # flat = np.reshape(self.goal_ids,(-1))
        # return np.random.choice(flat,size=nums,replace=False)
        
        robot_goals = []
        for idx, gid in enumerate(self.goal_ids_new):
            if idx == 0:
                ids = np.random.choice(gid,size = (split),replace=False)         
            if idx == 1:
                ids = np.random.choice(gid,size = (nums-split),replace=False)
                
            for id in ids:
                    robot_goals.append(id)
        robot_goals = np.array(robot_goals).reshape(-1)
        # robot_goals = np.array([11,13,16,12,3,17])
        print(robot_goals)
        # robot_goals = [float(rg) for rg in robot_goals]
        return robot_goals
    
    
    def generate_pattern(self):
        self.pattern = 255*np.ones((340,670,3),dtype=np.uint8)
        rsize = 100
        self.c_goal = rospy.get_param("goal_id",default=100)        
        for i,goal in enumerate(self.goal_ids):
            for j,pose in enumerate(goal):
                x11 = 10 + j * 110
                y11 = 10 + i * 110
                
                points = np.array([[x11,y11],[x11+rsize,y11], [x11+rsize, y11+rsize], [x11, y11+rsize]], dtype=np.int32)
                if pose in self.robot_goals:
                    if pose == self.c_goal:
                        c_mat_idx = np.where(self.goal_ids == self.c_goal)
                        c_points, textloc = self.get_points(row=c_mat_idx[0][0],col=c_mat_idx[1][0])
                        # self.pattern = cv2.fillConvexPoly(self.pattern, points, (0,0,255))
                        self.pattern = cv2.fillConvexPoly(self.pattern, points, (255,255,255))
                        cv2.putText(self.pattern,str(self.c_goal),textloc,cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),thickness=2) 
                    else:
                        # self.pattern = cv2.fillConvexPoly(self.pattern, points, (0,0,255))
                        self.pattern = cv2.fillConvexPoly(self.pattern, points, (255,255,255))
                else:
                    self.pattern = cv2.fillConvexPoly(self.pattern, points, (0,255,0))
                    
                
                cv2.putText(self.pattern,str(pose),(x11+int(rsize/2), y11+int(rsize/2)),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),thickness=2)
     
    def get_points(self,row,col,rsize=100):
        x11 = 10 + col * 110
        y11 = 10 + row * 110
        points = np.array([[x11,y11],[x11+rsize,y11], [x11+rsize, y11+rsize], [x11, y11+rsize]], dtype=np.int32)
        textloc = (x11+int(rsize/2), y11+int(rsize/2))
        return points , textloc
    
    
    
                   
    def run_experiment(self):
        try: 
            
            print("Wating for input...")
            print("Current User Name: ", rospy.get_param("user_name"))
            print("Current mode: ", rospy.get_param("current_mode"))
            print("Grasp on: ",rospy.get_param("grasp_on"))
            print("Time delta: ",rospy.get_param("time_delta")," sec.")
            
            print("\n")
            # cv_image = CvBridge().imgmsg_to_cv2(self.user_interface,desired_encoding="bgr8")
            # self.show_current_goal()
            self.generate_pattern()
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
                
            elif cv_key == ord('1'):
                if rospy.get_param("show_all_goals",default=False):
                    rospy.set_param("show_all_goals",False)
                else:
                    rospy.set_param("show_all_goals",True)
                    
                
            if cv_key == ord('n'):
                # rospy.set_param("reinit_goals",True)
                rospy.set_param("new_mode",True)
                self.robot_goals = self.get_robot_goals()
                
            
            if cv_key == ord('g'):
                if rospy.get_param("grasp_on"):
                    self.grasp_flag = False
                else:
                    self.grasp_flag = True

            
            rospy.set_param("grasp_on",self.grasp_flag)
        
        except Exception as e:
            print(f"Run error: {e}")

if __name__ == '__main__':
    
    ui  =  USER_INT()
    # rospy.sleep(1)
    while not rospy.is_shutdown():
        rospy.set_param("robot_goals",ui.robot_goals.tolist())
        ui.run_experiment()
        rospy.sleep(1/20)