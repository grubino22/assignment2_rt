#! /usr/bin/env python

## @package assignment_2_2024
# \file get_last_target.py
# \brief A ROS service node to provide the last received target goal from the action server.
# \author Giuseppe Rubino
# \version 1.0
# \date 28/03/2025
#
# \details
# 
# Subscribes to:<BR>
#    * /reaching_goal/goal : Receives goal messages from the action server.
# 
# Uses action server:<BR>
#    * /reaching_goal : The action server that receives target goals.
#
# Description: <BR>
# This node subscribes to the /reaching_goal/goal topic to receive the latest target goal and stores it. It provides a service, /get_last_goal, that returns the last received goal information when requested. The service response contains the position (x, y, z) of the last target goal.
#
import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2_2024.srv import Last_Target, Last_TargetResponse

last_target = None  # Global variable to store the last received target goal.

## 
# \brief Callback function for the target message.
# \param msg The received PlanningActionGoal message containing the goal information.
#
# \return None
#
# This function is triggered whenever a new goal is received. It logs the received goal position (x, y, z).
def target_Callback(msg):
    global last_target # Store the new goal in the global variable
    last_target = msg
    position = msg.goal.target_pose.pose.position
    rospy.loginfo(f"Received a new goal:\nx = {position.x},\ny = {position.y},\nz = {position.z}")

## 
# \brief Service handler function for processing requests for the last received target.
# \param req The request for the last target goal.
#
# \return Last_TargetResponse containing the information of the last goal.
#
# This function checks if a target has been received. If no target is available, it logs a warning and returns an empty response.
# If a target is available, it returns the position of the last target goal.
def process_target_request(req):
    global last_target
    if last_target is None:
        rospy.logwarn("No target available")
        return Last_TargetResponse()
    target_info = last_target.goal.target_pose.pose
    return Last_TargetResponse(target_info)

## 
# \brief Main function to initialize the ROS node and handle the service.
#
# \return None
#
# This function initializes the ROS node, sets up the subscriber to listen for incoming target goals,
# and provides a service to request the last received goal. It continuously waits for incoming requests.
def service_last_goal():
    rospy.init_node('get_last_target')
    
    # Subscriber for the target goal messages from the action server
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_Callback)
    
    # Service that processes requests for the last received goal
    rospy.Service('/get_last_goal', Last_Target, process_target_request)
    rospy.loginfo("Service node started. Waiting for requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        service_last_goal()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted")

