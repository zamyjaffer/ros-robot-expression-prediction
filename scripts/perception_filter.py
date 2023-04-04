#!/usr/bin/env python3

import rospy
import random
from ros_robot_expression_prediction.msg import object_info
from ros_robot_expression_prediction.msg import human_info
from ros_robot_expression_prediction.msg import perceived_info

#using a calss so that I can subscribe to 2 topics without confusing the node
class PerceptionFilterNode:
    def __init__(self):
        #initialising the node
        rospy.init_node('perception_filter', anonymous=True)
        #publishing the percieved info topic
        self.pub = rospy.Publisher('perceived_info', perceived_info, queue_size=0)
        #subscribing to the object info topic
        rospy.Subscriber('object_info', object_info, self.object_callback)
        #subscribing to the human info topic
        rospy.Subscriber('human_info', human_info, self.human_callback)
        rospy.spin()

    #function to call object info and use its data
    def object_callback(self, msg):
        self.object_info = msg
        self.filter_information()

    #function to call human info and use its data
    def human_callback(self, msg):
        self.human_info = msg
        self.filter_information()

    #function to listen to get data from both messages and filted based on values
    def filter_information(self):
        if hasattr(self, 'object_info') and hasattr(self, 'human_info'):
            # randomly select which variables to set to 0
            id = self.object_info.id
            obj_size = self.object_info.object_size
            action = self.human_info.human_action
            expression = self.human_info.human_expression

            # Randomly filter information
            filter_type = random.randint(1, 8)
            if filter_type == 1:
                obj_size = 0
            elif filter_type == 2:
                action = 0
            elif filter_type == 3:
                expression = 0
            elif filter_type == 4:
                obj_size = 0
                action = 0
            elif filter_type == 5:
                obj_size = 0
                expression = 0
            elif filter_type == 6:
                action = 0
                expression = 0
            elif filter_type == 7:
                obj_size = 0
                action = 0
                expression = 0

            #publishing perceived info message
            perceived_msg = perceived_info()
            perceived_msg.id = id
            perceived_msg.object_size = obj_size
            perceived_msg.human_action = action
            perceived_msg.human_expression = expression
            rospy.loginfo(perceived_msg)
            self.pub.publish(perceived_msg)
            #print(perceived_msg)

if __name__ == '__main__':
    try:
        PerceptionFilterNode()
    except rospy.ROSInterruptException:
        pass    
