from re import L
import rospy
import random
import numpy as np
import cv2
import rospkg
import os
import time
import argparse


def generate_pattern(goal_id):
    pattern = 255*np.ones((340,670,3),dtype="uint8")
    robot_goals = np.random.choice(goal_id.reshape(-1),size=6,replace=False)    
    rsize = 100
    for i,goal in enumerate(goal_id):
        for j,pose in enumerate(goal):
            x11 = 10 + j * 110
            y11 = 10 + i*110
            points = np.array([[x11,y11],[x11+rsize,y11], [x11+rsize, y11+rsize], [x11, y11+rsize]], dtype=np.int32)
            if pose in robot_goals:
                pattern = cv2.fillConvexPoly(pattern, points, (0,0,255))
            else:
                pattern = cv2.fillConvexPoly(pattern, points, (0,255,0))
            cv2.putText(pattern,str(pose),(x11+int(rsize/2), y11+int(rsize/2)),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),thickness=2)

    return pattern, robot_goals



if __name__ == '__main__':


    parser =  argparse.ArgumentParser()
    parser.add_argument("--subject_name","-sm",type=str,default="shubham")
    parser.add_argument("--delay","-d",type=float,default="0.2")
    args = parser.parse_args()
    
    rospy.init_node("testing")
    ur5_intpro_dir = rospkg.RosPack().get_path("ur5_intpro")
    goal_id = np.array([[5,4,3,2,1,0],
                        [11,10,9,8,7,6],
                        [17,16,15,14,13,12]],dtype=np.int)

    np.random.seed(0)    
    pattern, robot_goals = generate_pattern(goal_id)
    
    
    while not rospy.is_shutdown():
        cv2.imshow("pattern",pattern)
        key = cv2.waitKey(1)
        if key == ord('n'):

            if not os.path.isdir(os.path.join(ur5_intpro_dir,"data",args.subject_name)):
                os.mkdir(os.path.join(ur5_intpro_dir,"data",args.subject_name))

            t = time.localtime()
            timestamp = time.strftime('-%b-%d-%Y_%H_%M_%S', t)       
            pattern_filename     = os.path.join(ur5_intpro_dir,"data",args.subject_name,"pattern"+timestamp+".png")
            robot_goals_filename = os.path.join(ur5_intpro_dir,"data",args.subject_name,"robot_goals"+timestamp+".npy")
            
            cv2.imwrite(pattern_filename,pattern)
            np.save(robot_goals_filename,robot_goals)
            
            pattern, robot_goals = generate_pattern(goal_id)

        if key == ord('0'):
            rospy.set_param("do_render",False)
            rospy.set_param("show_shadow",False)
            rospy.set_param("render_all",False)

        
        if key == ord('1'):
            rospy.set_param("do_render",True)
            rospy.set_param("render_all",True)
        
        if key == ord('2'):
            rospy.set_param("do_render",True)
            rospy.set_param("show_shadow",True)
    
        if key == ord('g'):
            if rospy.get_param("grasp_distance",default=0) == 105:
                rospy.set_param("grasp_distance",0)
                print("Setting grasp distance to 0")
                rospy.sleep(1)
            else:
                rospy.set_param("grasp_distance",105)
                print("Setting grasp distance to 105")
                rospy.sleep(1)

        if key == ord('a'):
            rospy.set_param("goal_ids",[int(i) for i in goal_id.reshape(-1)])

        if key == ord('h'):
            rospy.set_param("go_home",True)

        if key == ord("s"):
            rospy.set_param("delay",args.delay)
            rospy.set_param("goal_ids",[int(g) for g in robot_goals])
            for goal in robot_goals:
                rospy.set_param("goal_id",int(goal))
                rospy.set_param("move_to_next_primitive",True)
                tmp = 0
                while rospy.get_param("move_to_next_primitive"):
                    if tmp > 0:
                        continue
                    else:
                        print("waiting\r")
                        tmp += 1