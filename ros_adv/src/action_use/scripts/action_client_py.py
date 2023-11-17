#! /usr/bin/env python3

import rospy
import actionlib
from actionlib import SimpleActionClient, SimpleGoalState
from action_use.msg import *

def activate_cb():
    rospy.loginfo("connect to server!")
    
def done_cb(state: SimpleGoalState, result: TwoIntSumResult):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("successful! result = %.2f", result.result)
    else:
        rospy.loginfo("error")
    
def feedback_cb(feedback: TwoIntSumFeedback):
    rospy.loginfo("now is %.2f%%", feedback.percent * 100)

if __name__ == "__main__":
    rospy.init_node("action_client_py")
    
    sac = SimpleActionClient("actions", TwoIntSumAction)
    goal = TwoIntSumGoal()
    goal.a = 1
    goal.b = 10
    
    sac.wait_for_server()
    sac.send_goal(goal, done_cb, activate_cb, feedback_cb)
    
    rospy.spin()


