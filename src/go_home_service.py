#! /usr/bin/env python3

import rospy
from ur5_intpro.srv import GoHome,GoHomeRequest,GoHomeResponse
from ur5_intpro.msg import Ur5Joints as unityUr5
from irl_robots.msg import ur5Control, matrix, rows, ur5Joints, gSimpleControl
from std_msgs.msg import Float64

class Robot_Service:
    
    # @staticmethod
    def process(req):
        print(f"robot type: {req.robot_type}")        
        print(f"go home joints: {req.joints}")
        if req.robot_type == "real":
            msg = ur5Control()
            pub  = rospy.Publisher("/ur5/control",ur5Control,queue_size=1)
            msg.command = "movej"
            msg.jointcontrol = True
            msg.time = 2
            msg.values = req.joints
            pub.publish(msg)
            # rospy.sleep(1)
            counter = 0
            while not rospy.is_shutdown():
                rospy.sleep(1/req.wait)
                counter += 1
                if counter == 5:
                    break
        elif req.robot_type  == "unity":
            msg_type = unityUr5
            msg = unityUr5()
            pub  = rospy.Publisher(req.topic,msg_type,queue_size=1)
            msg.joints = req.joints
            
        
            counter = 0
            while not rospy.is_shutdown():
                pub.publish(msg)
                rospy.sleep(1/req.wait)
                counter += 1
                if counter == 5:
                    break
        
        elif req.robot_type  == "gazebo":
            msg_type = Float64
            msg = Float64()
            pubs= [rospy.Publisher("/joint_{}_position_controller/command".format(i),Float64,queue_size=1) for i in range(6)]
            for i in range(6):   
                pubs[i].publish(req.joints[i]) 
        flag = True        
        return GoHomeResponse(flag)
    
    # @staticmethod
    def go_home():
        rospy.init_node("robot_services",anonymous=True)
        service = rospy.Service("go_home",GoHome,Robot_Service.process)        
        rospy.spin()
        
        
if __name__ == "__main__":
    Robot_Service.go_home()