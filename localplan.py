#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2,cos,sin,exp
import math
import numpy as np
from copy import copy, deepcopy
PI =  3.1415926535897
gamma = 0.1
ang = 0.05235987756
r = 0.4
R1 = 0.6
R2 = 0.6
v1 = 0.1

class ultibot:
	def __init__(self):

		rospy.init_node('turtlebot_controller', anonymous = True)

		self.velocity_publisher1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
		self.velocity_publisher2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size = 10)
		self.velocity_publisher3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size = 10)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose,self.update_pose)
		self.pose_subscriber2 = rospy.Subscriber('/turtle2/pose',Pose,self.update_pose2)
		self.pose_subscriber3 = rospy.Subscriber('/turtle3/pose',Pose,self.update_pose3)

		self.pose = Pose()
		self.pose2 = Pose()
		self.pose3 = Pose()
		self.rate = rospy.Rate(10)

	def update_pose(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x,4)
		self.pose.y = round(self.pose.y,4)

	def update_pose2(self, data):
		self.pose2 = data
		self.pose2.x = round(self.pose2.x,4)
		self.pose2.y = round(self.pose2.y,4)
	
	def update_pose3(self, data):
		self.pose3 = data
		self.pose3.x = round(self.pose3.x,4)
		self.pose3.y = round(self.pose3.y,4)

	def euclidean_distance(self, pose_1, pose_2, radius =0):
		return (sqrt(pow(pose_2.x - pose_1.x,2) + pow(pose_2.y - pose_1.y,2)) - radius)

	def euclidean_distance2(self, y1, x1):
		return (sqrt(pow(y1 - self.pose.y,2) + pow(x1 - self.pose.x,2)))




	def linear_vel(self, goal_pose1, radius=0,constant=2):
		return constant * self.euclidean_distance(goal_pose1, self.pose)

	def cap(self, angle):
		if(angle>PI):
			angle=angle-2*PI
		if(angle<(-1*PI)):
			angle=angle+2*PI
		return angle

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

	def ret_angle(self, y, x, pose_2):
		return atan2(y - pose_2.y, x - pose_2.x)

	def angular_vel(self, goal_pose, constant=5):
		return constant * (self.cap(self.steering_angle(goal_pose) - (self.pose.theta)))

	def trial_prepath(self, pose1, arr):
		i=0
		cpose = Pose()
		cpose.x = self.pose.x
		cpose.y = self.pose.y
		theta1 = self.steering_angle(pose1)
		# print(arr)

		while(1):
			arr[i][0] = self.pose.x + (i+1)*0.05*cos(theta1)
			arr[i][1] = self.pose.y + (i+1)*0.05*sin(theta1)
			# print(arr[0][0]," ",arr[0][1])
			# print(arr[i][0],"  ",arr[i][1])
			cpose.x = arr[i][0]
			cpose.y = arr[i][1]
			if(self.euclidean_distance(pose1,cpose)<0.05):
				index = i 
				break
			i = i + 1
		# print(arr)
			
		y = 0
		while(y < index):
			print(arr[y][0],"  ",arr[y][1]," ",y)
			y = y + 1


		return index

	"""def give_l_indices(self, obs_pose, center_index, arr):

	def shift_theta_points(self, center_index, upper_index, below_index, arr):

	def shift_alfa_points(self, upper_index, uppermost_index, below_index, bottommost_index, arr):"""

	def evade_hopefully(self):

		vel_msg = Twist()
		vel_msg2 = Twist()
		vel_msg3 = Twist()
		self_pose = Pose()
		self_pose2 = Pose()
		self_pose3 = Pose()
		poser = Pose()

		goal_pose = Pose()
		goal_pose.x = input("Enter goal x: ")
		goal_pose.y = input("Enter goal y: ")
 
		arr = np.zeros((300,2))
		# print(arr)
		# arr[5][1] = 5
		# print(arr)
		# print (arr[5][0])
		index = self.trial_prepath(goal_pose, arr)
		arro = deepcopy(arr)
		print(index)
		i=0
		"""while(i<=index):
			print(arr[i][0],"  ",arr[i][1])
			i = i + 1"""
		ang_line = self.steering_angle(goal_pose)

		i = 0
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		vel_msg2.linear.x = 0
		vel_msg2.linear.y = 0
		vel_msg2.linear.z = 0

		vel_msg2.angular.x = 0
		vel_msg2.angular.y = 0
		vel_msg2.angular.z = 0

		vel_msg3.linear.x = 0
		vel_msg3.linear.y = 0
		vel_msg3.linear.z = 0

		vel_msg3.angular.x = 0
		vel_msg3.angular.y = 0
		vel_msg3.angular.z = 0


		while(1):

			if(i>index):
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0
				self.velocity_publisher1.publish(vel_msg)
				self.rate.sleep()
				rospy.spin()
				break
			knot1 = 0
			knot2 = 0
			while(self.euclidean_distance2(arr[i][1],arr[i][0])>0.01):

				d1 = sqrt(pow(self.pose2.y - self.pose.y,2)+pow(self.pose2.x - self.pose.x,2))

				if(d1<1.5*(r+R1)):
					t=0
					k=i-15
					ang = abs(self.cap(self.ret_angle(arro[k][1],arro[k][0],self.pose2)-ang_line-PI/2))
					ang_prev = ang
					while(1):
						#print(k)
						#print(arr[k][1],"  ",arr[k][0])
						#print(self.cap(self.ret_angle(arr[k][1],arr[k][0],self.pose2)))
						#print(ang)
						if(k-i>30):
							knot1 = 1
							break
						k = k+1
						ang_prev = ang
						ang = abs(self.cap(self.ret_angle(arro[k][1],arro[k][0],self.pose2)-ang_line-PI/2))
						if(ang > ang_prev):
							k=k-1
							break
						

					j=i
					while(knot1!=1):
						if(j>(i+2*(k-i))):
							break
						ang = self.ret_angle(arr[j][1],arr[j][0],self.pose2)
						dis = sqrt(pow(arr[j][1] - self.pose2.y,2) + pow(arr[j][0] - self.pose2.x,2))
						if(dis<R1+r):
							arr[j][1] = self.pose2.y + (R1+r)*sin(ang)
							arr[j][0] = self.pose2.x + (R1+r)*cos(ang)
						j = j + 1

				else:
					t=1

				d2 = sqrt(pow(self.pose3.y - self.pose.y,2)+pow(self.pose3.x - self.pose.x,2))

				if(d2<1.5*(r+R2)):
					c=0
					k = i-15
					ang = abs(self.cap(self.ret_angle(arro[k][1],arro[k][0],self.pose3)-ang_line-PI/2))
					ang_prev = ang
					while(1):
						if(k-i>30):
							knot2 = 1
							break
						k = k+1
						ang_prev = ang
						ang = abs(self.cap(self.ret_angle(arro[k][1],arro[k][0],self.pose3)-ang_line-PI/2))
						if(ang > ang_prev):
							k=k-1
							break
						

					j=i
					while(knot2!=1):
						if(j>(i+2*(k-i))):
							break
						ang = self.ret_angle(arr[j][1],arr[j][0],self.pose3)
						dis = sqrt(pow(arr[j][1] - self.pose3.y,2) + pow(arr[j][0] - self.pose3.x,2))
						if(dis<R2+r):
							arr[j][1] = self.pose3.y + (R2+r)*sin(ang)
							arr[j][0] = self.pose3.x + (R2+r)*cos(ang)
						j = j+1

				else:
					c=1

				if(c==1 and t==1):
					arr = deepcopy(arro)

				poser.x = arr[i][0]
				poser.y = arr[i][1]
				vel_msg.linear.x = v1
				vel_msg.angular.z = self.angular_vel(poser)
				self.velocity_publisher1.publish(vel_msg)
				vel_msg2.linear.x = 0.4 
				vel_msg2.angular.z =  0.25
				self.velocity_publisher2.publish(vel_msg2)
				vel_msg3.linear.x = 0.4
				vel_msg3.angular.z = 0.25
				self.velocity_publisher3.publish(vel_msg3)

				self.rate.sleep()	

			i = i + 1

if __name__ == '__main__':
	try:
		x = ultibot()
		x.evade_hopefully()
	except rospy.ROSInterruptException:
		pass

							

							


						

























