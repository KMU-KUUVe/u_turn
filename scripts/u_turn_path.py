#!/usr/bin/env python
import rospy
import math
import actionlib

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback
from u_turn.msg import u_turnAction, u_turnGoal, u_turnFeedback


#front x +
#left y +
class u_turn:
	def __init__(self):
		rospy.init_node('u_turn', anonymous=True)

		self.pub = rospy.Publisher('ackermann', AckermannDriveStamped, queue_size=10)		
		#self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)
		
		# Client
		self.client = actionlib.SimpleActionClient('crosswalk_stop', u_turnAction)
		self.goal = u_turnGoal()

		self.server = actionlib.SimpleActionServer('u_turn_and_crosswalk_stop', MissionPlannerAction, execute_cb=self.execute_cb, auto_start=False)
		self.server.start()

		self.max_theta = rospy.get_param("/u_turn/max_theta", 45)
		self.throttle = rospy.get_param("/u_turn/throttle", 0)	
		self.lateral_offset = rospy.get_param("/u_turn/lateral_offset", 3.0)
		self.theta_error_factor = rospy.get_param("/u_turn/theta_error_factor", 1.0)
		self.lateral_error_factor = rospy.get_param("/u_turn/lateral_error_factor", 1.0)
		self.right_steer_scale = rospy.get_param("/u_turn/right_steer_scale", 2.0)	
		self.left_steer_offset = rospy.get_param("/u_turn/left_steer_offset", 3)

		self.start_flag = False
		self.finish_flag = False

	def updateParam(self):	
		self.max_theta = rospy.get_param("/u_turn/max_theta")	
		self.lateral_offset = rospy.get_param("/u_turn/lateral_offset")
		self.theta_error_factor = rospy.get_param("/u_turn/theta_error_factor")
		self.lateral_error_factor = rospy.get_param("/u_turn/lateral_error_factor")

	def crosswalk_done_cb(self, state, result):
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = 0
		acker_data.drive.steering_angle = 0
		self.pub.publish(acker_data)

		self.finish_flag = True

	# LiDAR Algorithm Start
	def execute_cb(self, goal):
		# find server!
		self.client.wait_for_server()
		# send goal to cross walk node
		self.client.send_goal(self.goal, done_cb=self.crosswalk_done_cb)
		# run algotihm
		self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)

		result = MissionPlannerResult()

		self.start_flag = True	

		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if(self.finish_flag == True):
				self.start_flag = False
				self.server.set_succeeded(result)	
				break
			r.sleep()
		

	def obstacles_cb(self, data):
		if(self.start_flag == True):
			self.updateParam()
			theta = 0.0
			gradient = 0.0
			acker_data = AckermannDriveStamped()

			#self.client.wait_for_server()
			#self.client.send_goal(self.goal)
			#self.client.wait_for_result()

			'''
			if self.detect_crosswalk given Result, acker_data.drive.speed = 0  
			'''	
			first_point = Point(0, 0, 0)
			last_point = Point(0, 0, 0)

			for segment_data in data.segments:
				first_point.x = first_point.x + segment_data.first_point.x
				last_point.x = last_point.x + segment_data.last_point.x
				first_point.y = first_point.y + segment_data.first_point.y
				last_point.y = last_point.y + segment_data.last_point.y
			first_point.x = first_point.x/len(data.segments)
			first_point.y = first_point.y /len(data.segments)
			last_point.x = last_point.x /len(data.segments)
			last_point.y = last_point.y /len(data.segments)

			print("first_point: " + str(first_point))
			print("last_point: " + str(last_point))

			if(last_point.x == first_point.x):
				acker_data.drive.steering_angle = -26
			else:
				rospy.loginfo
				gradient = (last_point.y - first_point.y)/(last_point.x - first_point.x)
				theta = (math.atan(gradient)*180)/math.pi

				# ax+by+c = 0
				a = gradient
				b = -1
				c = -gradient*first_point.x + first_point.y

				lateral = abs(c)/math.sqrt(a**2 + b**2)
				
				acker_data.drive.steering_angle = -(theta*26/self.max_theta) * self.theta_error_factor + (lateral - self.lateral_offset) * self.lateral_error_factor
				acker_data.drive.steering_angle = int(acker_data.drive.steering_angle)

				print("theta error: " + str(-(theta*26/self.max_theta)))
				print("lateral error: " + str(lateral - self.lateral_offset))
			
				if (acker_data.drive.steering_angle > 0):
				    acker_data.drive.steering_angle = int(acker_data.drive.steering_angle/self.right_steer_scale)
				elif (acker_data.drive.steering_angle < 0):
				    acker_data.drive.steering_angle = acker_data.drive.steering_angle - self.left_steer_offset

				if (acker_data.drive.steering_angle > 26):  # max steering
					acker_data.drive.steering_angle = 26
				elif (acker_data.drive.steering_angle < -26):
					acker_data.drive.steering_angle = -26
				else:
					pass


			acker_data.drive.speed = self.throttle
					
			print("speed : " + str(acker_data.drive.speed))
			print("steering : " + str(acker_data.drive.steering_angle))
			print("-----------------------------------")

			# don't send messages if detect a crosswalk line.
			if self.finish_flag== False:
				self.pub.publish(acker_data)
		
		
if __name__ == '__main__':
	try:
		u_turn_mission = u_turn()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		print(error)
		pass			
				
	
