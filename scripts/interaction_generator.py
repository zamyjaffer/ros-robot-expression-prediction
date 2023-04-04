#!/usr/bin/env python3

import rospy
import random
from ros_robot_expression_prediction.msg import object_info
from ros_robot_expression_prediction.msg import human_info

def interaction_generator():
    #creating publishers for both topics
    object_pub = rospy.Publisher('object_info', object_info, queue_size=0)
    human_pub = rospy.Publisher('human_info', human_info, queue_size=0)
    #initialising the node
    rospy.init_node('interaction_generator', anonymous=True)
    rate = rospy.Rate(1/10) #once every 10 seconds
    interaction_id = 1
    
    while not rospy.is_shutdown():
        #generating random ints
        object_size = random.randint(1, 2)
        human_expression = random.randint(1, 3)
        human_action = random.randint(1, 3)
        
        #creating object message
        object_msg = object_info()
        object_msg.id = interaction_id
        object_msg.object_size = object_size
        rospy.loginfo(object_msg)
        object_pub.publish(object_msg)
        
        #creating human message
        human_msg = human_info()
        human_msg.id = interaction_id
        human_msg.human_expression = human_expression
        human_msg.human_action = human_action
        rospy.loginfo(human_msg)
        human_pub.publish(human_msg)
        
        #incrementing the ID
        interaction_id += 1
        #delay
        rate.sleep()

if __name__ == '__main__':
    try:
        interaction_generator()
    except rospy.ROSInterruptException:
        pass    
