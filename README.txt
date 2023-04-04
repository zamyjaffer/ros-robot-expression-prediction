ROS Robot Expression Predictions

###################
### Description ###
###################
- This is a ROS project that creates a bayesian network to predict a robots expression when interacting with a child based on information about the human and the object. This project consists of 4 nodes:
  1. 'interaction_generator' which generates random information for a human/robot interaction every 10 seconds, including; an ID, object size, human expression, and human action.
  2. 'perception_filter' which recieves the parameters from the first node and filters the data randomly, setting some variables to 0. 
  3. 'robot_controller' which subscribes to the topics from the second node and calls node 4 to calculate the probabilities of the robots expressions.
  4. 'robot_expression_prediction' which implements the 'predict_robot_expression service and uses a Bayesian network to predict the probabilities of the robots expression given the object size, human action or human expression.
- This project also consists of a launch file that starts the nodes and shows graph of this project using rqt_graph.

###################
## Installation  ##
###################
- Download the 'ros_robot_expression_prediction' package into your catkin workspace 
- From the catkin workspace run the following command: 
    catkin_make

###################
##### Running #####
###################
- from your catkin_workspace activate roscore: 
    roscore

- from your catkin_workspace navigate to the scripts folder: 
    cd /src/ros_robot_expression_prediction/scripts
    
- run the following commands from within the srcipts folder for each node:
    chmod +x interaction_generator.py
    chmod +x perception_filter.py
    chmod +x robot_controller.py
    chmod +x robot_expression_prediction.py
    
- navigate back to the catkin_ws folder: 
    cd ~/ catkin_ws
    
- launch the package:
    roslaunch ros_robot_expression_prediction human_robot_interaction.launch
    
###################
## Dependencies  ##
###################
- python 3.8.10
- ros-noetic
- random
