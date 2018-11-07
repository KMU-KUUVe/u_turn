#!/usr/bin/env python

import roslib; roslib.load_manifest('crosswalk')
import rospy
import actionlib

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point

from u_turn import u_trun
		
def execute_cb(goal):
	rospy.loginfo("Goal Received")
	u_turn_mission = u_turn()
	result = 
	result.time_elapsed = rospy.Duration(1)
	u_turn_mission.execute()
	action_server.set_succeeded(result)
		
if __name__ == '__main__':
	rospy.init_node('u_turn', anonymous=True)
	try:
		action_name = 'u_turn'
		client = actionlib.SimpleActionServer(action_name, , execute_cb=execute_cb, auto_start=False)
		action_server.start()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		print(error)
		pass			
				
	
