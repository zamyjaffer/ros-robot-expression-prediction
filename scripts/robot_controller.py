#!/usr/bin/env python3

import rospy
import random 
from ros_robot_expression_prediction.msg import perceived_info
from ros_robot_expression_prediction.msg import robot_info
from ros_robot_expression_prediction.srv import *

#using a calss so that I can subscribe to 2 topics without confusing the node
class RobotController:
    
    def __init__(self):
        #initialising the node
        rospy.init_node('robot_controller')
        #waiting for the perdict robot expression service to start
        rospy.wait_for_service('predict_robot_expression')
        #creating a proxy for the service
        self.predict_robot_expression = rospy.ServiceProxy('predict_robot_expression', predict_robot_expression)
        #subscribing to the percieve info topic
        self.percieve_sub = rospy.Subscriber('perceived_info', perceived_info, self.perceived_info_callback)
        #publishing the robot info topic
        self.robot_info_pub = rospy.Publisher('robot_info', robot_info, queue_size=0)

    def perceived_info_callback(self, msg):
        #initialising probabilities
        p_happy, p_sad, p_neutral = 0.0, 0.0, 0.0
        
        id = msg.id
        
        #getting probabilities for each robot expression from the service
        if msg.object_size != 0 and msg.human_action != 0 and msg.human_expression != 0:
            try:
                #getting the response
                response = self.predict_robot_expression(msg.object_size, msg.human_action, msg.human_expression)
                #assigining variables
                p_happy, p_sad, p_neutral = response.p_happy, response.p_sad, response.p_neutral
                #logging probabilities
                rospy.loginfo("Happy: %f, Sad: %f, Neutral: %f", p_happy, p_sad, p_neutral)
                
                #publishing robot info message
                robot_info_msg = robot_info()
                robot_info_msg.id = id
                robot_info_msg.p_happy = p_happy
                robot_info_msg.p_sad = p_sad
                robot_info_msg.p_neutral = p_neutral
                self.robot_info_pub.publish(robot_info_msg)
                
            except rospy.ServiceException as e:
                print("Service call failed: ", e)




if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
