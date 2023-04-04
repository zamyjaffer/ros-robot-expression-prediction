#!/usr/bin/env python3

import rospy
from ros_robot_expression_prediction.srv import *
from ros_robot_expression_prediction.msg import perceived_info

#using a calss to work with all the different variables
class BayesNet:
    def __init__(self):
        #creating dictionaries based on the probabilities given for the assignment
        self.pO = {'small': 1/2, 'big': 1/2}
        self.pHA = {'robot': 1/3, 'object': 1/3, 'away': 1/3}
        self.pHE = {'happy': 1/3, 'sad': 1/3, 'neutral': 1/3}
        self.pRE = {
            ('happy', 'robot', 'small'): {'happy': 0.8, 'sad': 0.2, 'neutral': 0.0},
            ('happy', 'robot', 'big'): {'happy': 1.0, 'sad': 0.0, 'neutral': 0.0},
            ('happy', 'object', 'small'): {'happy': 1.0, 'sad': 0.0, 'neutral': 0.0},
            ('happy', 'object', 'big'): {'happy': 1.0, 'sad': 0.0, 'neutral': 0.0},
            ('happy', 'away', 'small'): {'happy': 0.6, 'sad': 0.2, 'neutral': 0.2},
            ('happy', 'away', 'big'): {'happy': 0.8, 'sad': 0.2, 'neutral': 0.0},
            ('sad', 'robot', 'small'): {'happy': 0.0, 'sad': 0.0, 'neutral': 1.0},
            ('sad', 'robot', 'big'): {'happy': 0.0, 'sad': 0.0, 'neutral': 1.0},
            ('sad', 'object', 'small'): {'happy': 0.0, 'sad': 0.1, 'neutral': 0.9},
            ('sad', 'object', 'big'): {'happy': 0.1, 'sad': 0.1, 'neutral': 0.8},
            ('sad', 'away', 'small'): {'happy': 0.0, 'sad': 0.2, 'neutral': 0.8},
            ('sad', 'away', 'big'): {'happy': 0.2, 'sad': 0.2, 'neutral': 0.6},
            ('neutral', 'robot', 'small'): {'happy': 0.7, 'sad': 0.3, 'neutral': 0.0},
            ('neutral', 'robot', 'big'): {'happy': 0.8, 'sad': 0.2, 'neutral': 0.0},
            ('neutral', 'object', 'small'): {'happy': 0.8, 'sad': 0.2, 'neutral': 0.0},
            ('neutral', 'object', 'big'): {'happy': 0.9, 'sad': 0.1, 'neutral': 0.0},
            ('neutral', 'away', 'small'): {'happy': 0.6, 'sad': 0.2, 'neutral': 0.2},
            ('neutral', 'away', 'big'): {'happy': 0.7, 'sad': 0.2, 'neutral': 0.1},
        }
    
    def predict(self, object_size, human_action, human_expression):
        #initialising the probabilities
        p_happy, p_sad, p_neutral = 0.0, 0.0, 0.0
        print(object_size, human_action, human_expression)
        #where all variables are known
        if(object_size != 0 and human_expression != 0 and human_action != 0):
            #get the probabilities from the dictionary and update the variables
            result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
            p_happy, p_sad, p_neutral = result["happy"], result["sad"], result["neutral"]
        
            
        #where object_size is not known
        elif(object_size == 0 and human_expression != 0 and human_action != 0):
            #loop through the number of values in object_size and calculate the probabilities given the data I have for the other probabilities using conditional probability & chain rule
            for i in range(1,3):
                object_size = i
                probO  = self.pO.get((['small', 'big'][object_size-1]), None)
                result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
                #update probabilities
                p_happy += result["happy"] * probO 
                p_sad += result["sad"] * probO
                p_neutral += result["neutral"] * probO
        
        #where human_expression is not known
        elif(object_size != 0 and human_expression == 0 and human_action != 0):
            #loop through the number of values in human_expression and calculate the probabilities given the data I have for the other probabilities using conditional probability & chain rule
            for i in range(1,4):
                human_expression = i
                probHE  = self.pHE.get((['happy', 'sad', 'neutral'][human_expression-1]), None)
                result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
                #update probabilities
                p_happy += result["happy"] * probHE
                p_sad += result["sad"] * probHE
                p_neutral += result["neutral"] * probHE
                
        #where human_action is not known
        elif(object_size != 0 and human_expression != 0 and human_action == 0):
            #loop through the number of values in human_action and calculate the probabilities given the data I have for the other probabilities using conditional probability & chain rule
            for i in range(1,4):
                human_action = i
                probHA  = self.pHA.get((['robot', 'object', 'away'][human_action-1]), None)
                result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
                #update probabilities
                p_happy += result["happy"] * probHA
                p_sad += result["sad"] * probHA
                p_neutral += result["neutral"] * probHA
                
        #where only object_size is known        
        elif (object_size != 0 and human_expression == 0 and human_action == 0):
            #loop through the number of values in human_expression and human_action and calculate the probabilities given the data I have for the other probabilities using conditional probability & chain rule
            for i in range(1,4):
                human_expression = i
                for j in range(1,4):
                    human_action = j
                    result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
                    #update probabilities
                    p_happy += result["happy"]
                    p_sad += result["sad"]
                    p_neutral += result["neutral"]
            #find values in dictionary
            prob_human_express  = self.pHE.get((['happy', 'sad', 'neutral'][human_expression-1]), None)
            prob_human_act  = self.pHA.get((['robot', 'object', 'away'][human_action-1]), None)
            #update probabilities
            p_happy *= prob_human_express * prob_human_act
            p_sad *= prob_human_express * prob_human_act
            p_neutral *= prob_human_express * prob_human_act
  	    
        #where only human_expression is known
        elif(object_size == 0 and human_expression != 0 and human_action == 0):
            #loop through the number of values in object_size and human_action and calculate the probabilities given the data I have for the other probabilities using conditional probability & chain rule
            for i in range(1,3):
                object_size = i
                for j in range(1,4):
                    human_action = j
                    result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
                    #update probabilities
                    p_happy += result["happy"]
                    p_sad += result["sad"]
                    p_neutral += result["neutral"] 
            #find values in dictionary
            prob_obj_size  = self.pO.get((['small', 'big'][object_size-1]), None)
            prob_human_act  = self.pHA.get((['robot', 'object', 'away'][human_action-1]), None)
            #update probabilities
            p_happy *= prob_obj_size * prob_human_act
            p_sad *= prob_obj_size * prob_human_act
            p_neutral *= prob_obj_size * prob_human_act
        
        #where only human_action is known
        elif(object_size == 0 and human_expression == 0 and human_action != 0):
            #loop through the number of values in object_size and human_expression and calculate the probabilities given the data I have for the other probabilities using conditional probability & chain rule
            for i in range(1,3):
                object_size = i
                for j in range(1,4):
                    human_expression = j
                    result = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1], ['robot', 'object', 'away'][human_action-1], ['small', 'big'][object_size-1]), None)
                    #update probabilities
                    p_happy += result["happy"]
                    p_sad += result["sad"]
                    p_neutral += result["neutral"]
            #find values in dictionary
            prob_human_express  = self.pRE.get((['happy', 'sad', 'neutral'][human_expression-1]), None)
            #update probabilities
            prob_obj_size  = self.pO.get((['small', 'big'][object_size-1]), None)
            p_happy *= prob_human_express * prob_obj_size
            p_sad *= prob_human_express * prob_obj_size
            p_neutral *= prob_human_express * prob_obj_size
	    
        return p_happy, p_sad, p_neutral

class RobotExpressionPredictor:
    def __init__(self):
        #initialising the bayesian network
        self.bayes_network = BayesNet()

        #initialising the node
        rospy.init_node('robot_expression_prediction')
        #initialising the service
        self.service = rospy.Service('predict_robot_expression', predict_robot_expression, self.handle_request)
        rospy.loginfo('Ready to predict robot expression')
        #subscribing to the perceived info topic
        self.subscriber = rospy.Subscriber('perceived_info', perceived_info, self.handle_perceived_info)
        #initialising the current observation
        self.current_observation = {
            'object_size': 0,
            'human_action': 0,
            'human_expression': 0
        }

    def handle_request(self, request):
    	#predicting the robot expression using bayesian network
        p_happy, p_sad, p_neutral = self.bayes_network.predict(self.current_observation['object_size'], self.current_observation['human_action'],  self.current_observation['human_expression'])
        
        rospy.loginfo('val', self.current_observation)

        return p_happy, p_sad, p_neutral

    def handle_perceived_info(self, msg):
        #updating the current observation with the perceived info
        self.current_observation['object_size'] = msg.object_size
        self.current_observation['human_action'] = msg.human_action
        self.current_observation['human_expression'] = msg.human_expression
        
if __name__ == '__main__':
    predictor = RobotExpressionPredictor()
    rospy.spin()
