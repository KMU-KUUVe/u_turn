#!/usr/bin/env python
import rospy
import actionlib

from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback
from u_turn_path import u_turn


def execute_cb(goal):
	rospy.loginfo("Goal Received")
	result = MissionPlannerResult()
	u_turn_mission = u_turn()
	action_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('u_turn', anonymous=True)
    try:
        action_name = 'u_turn'
        action_server = actionlib.SimpleActionServer(action_name,MissionPlannerAction, execute_cb=execute_cb, auto_start=False)
        action_server.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(error)
	pass
