#! /usr/bin/env python
import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2_2024.srv import Last_Target, Last_TargetResponse

last_target = None
    
def target_Callback(msg):
    global last_target
    last_target = msg
    position = msg.goal.target_pose.pose.position
    rospy.loginfo(f"Received a new goal:\nx = {position.x},\ny = {position.y},\nz = {position.z}")
  
def process_target_request(req):
    global last_target
    if last_target is None:
        rospy.logwarn("No target available")
        return Last_TargetResponse(success=False)
    target_info = last_target.goal.target_pose
    return Last_TargetResponse(target_info, success=True)
    
def service_last_goal():
    rospy.init_node('get_last_target')

    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, target_Callback)

    rospy.Service('/get_last_goal', Last_Target, process_target_request)
    rospy.loginfo("Service node started. Waiting for requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        service_last_goal()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted")

