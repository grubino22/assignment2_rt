#! /usr/bin/env python
import rospy
import actionlib
from actionlib import GoalStatus
import assignment_2_2024.msg
from assignment_2_2024.msg import robot_par
from nav_msgs.msg import Odometry

def feedback_Callback(feedback):
    if feedback.stat == "Target reached!":
    	rospy.loginfo(f"Received feedback: \n{feedback.actual_pose} \nStatus: {feedback.stat}")
    	print("\nYou may set a new goal or quit.\nEnter 'set' to set a new goal, 'c' to cancel, or 'q' to quit:")

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

def odom_Callback(msg):
    robot_param = robot_par()
    robot_param.x = msg.pose.pose.position.x
    robot_param.y = msg.pose.pose.position.y
    robot_param.vel_x = msg.twist.twist.linear.x
    robot_param.vel_z = msg.twist.twist.angular.z

    pub_pos_vel.publish(robot_param)

def get_coordinates():
    while True:
        try:
            x = float(input("Enter the x coordinate: "))
            y = float(input("Enter the y coordinate: "))
            return x, y
        except ValueError:
            rospy.logwarn("Invalid input, please enter only numbers!")

def main():
    global pub_pos_vel
    global client

    rospy.init_node('set_target')
    
    pub_pos_vel = rospy.Publisher('/robot_position_velocity', robot_par, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_Callback)
    
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

