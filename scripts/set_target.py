#! /usr/bin/env python


## @package assignment_2_2024
# \file set_target.py
# \brief A ROS client node to set and cancel goals for a robot using an action server.
# \author Giuseppe Rubino
# \version 1.0
# \date 28/03/2025
#
# \details
# 
# Publishes to:<BR>
#    * /robot_position_velocity : Publishes the robot's current position and velocity.
# 
# Subscribes to:<BR>
#    * /odom : Receives odometry data from the robot.
# 
# Uses action server:<BR>
#    * /reaching_goal : Sends goal position to be reached by the robot.
#
# Description: <BR>
# This node allows the user to set goals, cancel them, and track the robot position and velocity.
# The action client communicates user requests to the action server. When a new goal is set, the server processes the request and commands the robot to navigate towards the specified coordinates. If the user opts to cancel a goal, the server first checks if an active or pending goal exists. If found, it terminates the goal and informs the client of the cancellation.
#

import rospy
import actionlib
from actionlib import GoalStatus
import assignment_2_2024.msg
from assignment_2_2024.msg import robot_par
from nav_msgs.msg import Odometry

pub_pos_vel = None # Global variable for the publisher of robot position and velocity, publishes a custom message `robot_par` containing position and velocity data.

client = None # Global variable for the action client, used to send and cancel goals to the `/reaching_goal` action server.

## 
# \brief Callback function for feedback from the action server.
# \param feedback The feedback message containing the robot's actual pose and status.
#
# \return None
#
# Logs the robot's current pose and status when the target is reached. Prompts the user to set a new goal or quit.
def feedback_Callback(feedback):
    if feedback.stat == "Target reached!":
    	rospy.loginfo(f"Received feedback: \n{feedback.actual_pose} \nStatus: {feedback.stat}")
    	print("\nYou may set a new goal or quit.\nEnter 'set' to set a new goal, 'c' to cancel, or 'q' to quit:")

## 
# \brief Sends a goal to the action server.
# \param client The SimpleActionClient instance.
# \param x The x-coordinate of the goal.
# \param y The y-coordinate of the goal.
#
# \return None
# 
# Sends a goal position (x, y) to the action server and waits for confirmation. If the server is not available, it logs an error message.
def set_goal(client, x, y):    
    rospy.loginfo("Waiting for action server availability")
    if not client.wait_for_server(timeout=rospy.Duration(10)):
        rospy.logerr("Action server not available.")
        return
    goal = assignment_2_2024.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    client.send_goal(goal, feedback_cb=feedback_Callback)
    rospy.loginfo("Goal sent, enter 'c' to cancel.")

## 
# \brief Cancels the currently active goal.
# \param client The SimpleActionClient instance.
# 
# \return None
# 
# Cancels the current goal if one is active or pending. If cancellation is successful, a confirmation message is displayed; otherwise, a warning is logged.
def cancel_goal(client):
    if client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Cancelling current goal")
        client.cancel_goal()
        rospy.sleep(0.5)
        state = client.get_state()
        if state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully cancelled\n\nYou may set a new goal or quit.\nEnter 'set' to set a new goal, 'c' to cancel, or 'q' to quit")
        else:
            rospy.logwarn("Failed to cancel the goal")
    else:
        rospy.logwarn("No active goal to cancel.")

## 
# \brief Callback function for the odometry topic subscriber.
# \param msg The received Odometry message containing position and velocity information.
#
# \return None
#
# Extracts the robot's position (x, y) and velocity (linear and angular) from the Odometry message and publishes this data to the `/robot_position_velocity` topic.
def odom_Callback(msg):
    robot_param = robot_par()
    robot_param.x = msg.pose.pose.position.x
    robot_param.y = msg.pose.pose.position.y
    robot_param.vel_x = msg.twist.twist.linear.x
    robot_param.vel_z = msg.twist.twist.angular.z

    pub_pos_vel.publish(robot_param)

## 
# \brief Gets user input for goal coordinates.
#
# \return A tuple containing the x and y coordinates.
#
# Prompts the user to input valid numeric values for the x and y coordinates. Ensures only valid numeric inputs are accepted.
def get_coordinates():
    while True:
        try:
            x = float(input("Enter the x coordinate: "))
            y = float(input("Enter the y coordinate: "))
            return x, y
        except ValueError:
            rospy.logwarn("Invalid input, please enter only numbers!")

## 
# \brief Main function.
#
# \return None
#
# This function initializes the ROS node, sets up publishers and subscribers, and continuously listens for user commands to set or cancel goals.
def main():
    global pub_pos_vel
    global client

    rospy.init_node('set_target')
    
    # Publisher for robot position and velocity
    pub_pos_vel = rospy.Publisher('/robot_position_velocity', robot_par, queue_size=10)
    
    # Subscriber to odometry topic
    rospy.Subscriber('/odom', Odometry, odom_Callback)
    
    # Action client to send goals
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
    
    while not rospy.is_shutdown():
        rospy.loginfo_once("Enter 'set' to set a goal, 'c' to cancel the current goal, or 'q' to quit:")
        answer = input().strip().lower()
        if answer == 'set':
            x, y = get_coordinates()
            set_goal(client, x, y)
        elif answer == 'c':
            cancel_goal(client)
        elif answer == 'q':
            rospy.loginfo("Exiting the client")
            break
        else:
            rospy.logwarn("Invalid command. Please try again")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)

