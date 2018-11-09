#!/usr/bin/env python
import rospy
import actionlib

from u_turn.msg import u_turnAction, u_turnGoal, u_turnFeedback

class U_turn_Client:
	def __init__(self):
		rospy.init_node("u_turn_client")
		self.client = actionlib.SimpleActionClient('cross_walk_detect_goal', u_turnAction)
		self.speed = 0
		self.goal = u_turnGoal()
		self.crosswalk_detected = False

	def execute(self):
		self.client.wait_for_server()
		
		# choose feedback or result
		self.client.send_goal(goal)
		self.client.wait_for_result()

		self.crosswalk_detected = True


if __name__ == '__main__':
	try:
		
		u_turn_client = U_turn_Client()
		u_turn_client.execute()
		rospy.spin()
		

	except rospy.ROSInterruptException:
		print(error)
		pass	
		
