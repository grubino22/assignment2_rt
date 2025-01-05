#! /usr/bin/env python
import rospy
import actionlib
from actionlib import GoalStatus
import assignment_2_2024.msg
from assignment_2_2024.msg import robot_par
from nav_msgs.msg import Odometry

def odom_Callback(msg):
    robot_param = robot_par()
    robot_param.x = msg.pose.pose.position.x
    robot_param.y = msg.pose.pose.position.y
    robot_param.vel_x = msg.twist.twist.linear.x
    robot_param.vel_z = msg.twist.twist.angular.z

    pub_pos_vel.publish(robot_param)
        
def main():
    global pub_pos_vel
    global client

    rospy.init_node('reach_goal')
    
    # Publisher for postion and velocity
    pub_pos_vel = rospy.Publisher('/robot_position_velocity', robot_par, queue_size=10)
    # Subscriber to /odom topic for retrive the data from the robot
    rospy.Subscriber('/odom', Odometry, odom_Callback)
    
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
    
    while not rospy.is_shutdown():
        rospy.loginfo_once("Enter 'set' to set a goal, 'cancel' to cancel the current goal, or 'q' to quit")
        answer = input("Command (set=set goal, cancel=cancel goal, q=quit): ").strip().lower()
        if answer == 'set':
    	    valid_input = False
    	    while not valid_input:
    	    	try:
    	    		x = float(input("Enter the x coordinate: "))
    	    		y = float(input("Enter the y coordinate: "))
    	    		valid_input = True
    	    	except ValueError:
    	    		rospy.logwarn("Not valid input, only enter numbers!")
    	    set_goal(client, x, y)
        elif answer == 'cancel':
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
