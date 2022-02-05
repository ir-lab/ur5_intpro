


import rospy
from irl_robots.msg import ur5Joints
import time
import numpy as np
import random

CLOSING_DISTANCE = 105
OPENING_DISTANCE = 0

def pick_and_place_object(object_id=0):
    rospy.set_param("grasp_distance",OPENING_DISTANCE)
    rospy.set_param("go_over_object",False)
    rospy.set_param("place_object",False)
    rospy.set_param("go_to_object",False)
    rospy.set_param("go_to_midhome",False)
    rospy.set_param("finished_traj",False)
    rospy.set_param("finished_subtask",False)
    rospy.set_param("go_final",False)
    

    # try:
    rospy.set_param("go_over_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        print("waiting to finish traj for going over the object")
        time.sleep(0.1)
    rospy.set_param("go_over_object",False)
    rospy.set_param("finished_traj",False)
    

    rospy.set_param("go_to_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        print("waiting to finish traj for going to object")
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_to_object",False)


    rospy.set_param("grasp_distance",CLOSING_DISTANCE)
    # time.sleep(0.2)

    rospy.set_param("go_over_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        print("waiting to finish traj for going over the object")
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_over_object",False)
    
    
    rospy.set_param("place_object",False)
    rospy.set_param("go_final",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        print("waiting to finish traj for final way_point")
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_final",False)
    
    
    rospy.set_param("place_object",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        print("waiting to finish traj for going to place the object")
        time.sleep(0.1)            
    rospy.set_param("finished_traj",False)
    rospy.set_param("place_object",False)
    
    
    rospy.set_param("grasp_distance",OPENING_DISTANCE)
    
    rospy.set_param("place_object",False)
    rospy.set_param("go_final",True)
    rospy.set_param("move_to_next_primitive",True)
    while not rospy.get_param("finished_traj"):
        print("waiting to finish traj for final way_point")
        time.sleep(0.1)
    rospy.set_param("finished_traj",False)
    rospy.set_param("go_final",False)
    

    # rospy.set_param("go_to_midhome",True)
    # rospy.set_param("move_to_next_primitive",True)
    # while not rospy.get_param("finished_traj"):
    #     print("waiting to finish traj for midhome")
    #     time.sleep(0.1)
        
    rospy.set_param("finished_subtask",True)
    # except KeyboardInterrupt:
    #     print("Setting default values to ros parameters")
    #     rospy.set_param("go_over_object",False)
    #     rospy.set_param("finished_traj",True)
    #     rospy.set_param("place_object",False)
    #     rospy.set_param("go_to_object",False)
        
    
    
if __name__ == '__main__':
    
    rospy.init_node("pick_and_place",anonymous=True)
    # goal_id = [0,1,2,3,4,5]
    goal_id = [0,1,2]
    random.shuffle(goal_id)
    if not rospy.is_shutdown():
        try:
            for id in goal_id:
                print("Goaing to id {}".format(id))
                rospy.set_param("goal_id",id)
                time.sleep(0.1)
                pick_and_place_object(object_id=id)
                
        except KeyboardInterrupt:
            print("Shuting down pick and place")