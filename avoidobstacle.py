#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, sqrt, atan2
PI = 3.1415926535897
import math

class mybot2:

	def __init__(self):

		rospy.init_node('turtlebot_controller',anonymous=True)

		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
		self.pose_subscriber1=rospy.Subscriber('/turtle1/pose',Pose, self.update_pose1)
		self.pose_subscriber2=rospy.Subscriber('/turtle2/pose',Pose, self.update_pose2)
		self.pose1 = Pose()
		self.pose2 = Pose()
		self.rate = rospy.Rate(10)

	def update_pose1(self, data):
		self.pose1 = data
		self.pose1.x = round(self.pose1.x, 4)
		self.pose1.y = round(self.pose1.y, 4)

	def update_pose2(self, data):
		self.pose2 = data
		self.pose2.x = round(self.pose2.x, 4)
		self.pose2.y = round(self.pose2.y, 4)

	def euclidean_distance(self, goal_pose, radius):
		return (sqrt(pow((goal_pose.x - self.pose1.x), 2) + pow((goal_pose.y - self.pose1.y), 2)) - radius)

	def cap(self, angle):
		if(angle>PI):
			angle=angle-2*PI
		if(angle<(-1*PI)):
			angle=angle+2*PI
		return angle


	def linear_vel(self, goal_pose, radius,constant=1.1):
		return constant * self.euclidean_distance(goal_pose, radius)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose1.y, goal_pose.x - self.pose1.x)

	def angular_vel(self, goal_pose, constant=2):
		return constant * (self.cap(self.steering_angle(goal_pose) - (self.pose1.theta)))

	def reachborder(self):
		vel_msg = Twist()
		clearance = input("Enter clearance :")
		goal_pose=Pose()

		while(1):
			print(self.pose1)
			print(self.pose2)
			ch=0
			if(abs(self.cap(self.pose1.theta) - self.cap(self.steering_angle(self.pose2)))==0):
				continue
			ch=1
			theta = self.cap(self.steering_angle(self.pose2))

			while (abs(self.cap(self.pose1.theta) - self.cap(self.steering_angle(self.pose2)))>0.01):
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel(self.pose2)
				print(vel_msg.angular.z)
				self.velocity_publisher.publish(vel_msg)
				self.rate.sleep()
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)

			while self.euclidean_distance(self.pose2, 0) >clearance:


				vel_msg.linear.x = self.linear_vel(self.pose2, 0)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel(self.pose2)

				self.velocity_publisher.publish(vel_msg)

				self.rate.sleep()

			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)

			a=1;
			n=PI/0.1
			thetanot= self.cap(self.steering_angle(self.pose2))
			
			while(a<n):
				
				halfway = 0
				cir_pose = Pose()
				cir_pose.x = self.pose2.x - clearance*math.cos(self.cap(thetanot-a*0.1))
				cir_pose.y = self.pose2.y - clearance*math.sin(self.cap(thetanot-a*0.1))
				dis = self.euclidean_distance(cir_pose,0)
	
				while (abs(self.cap(self.steering_angle(cir_pose)) - self.cap(self.pose1.theta))>0.01):

					if(self.euclidean_distance(cir_pose,0)<=0.5*dis):
						halfway=1
						break
					vel_msg.linear.x = self.linear_vel(cir_pose, 0)
					vel_msg.linear.y = 0
					vel_msg.linear.z = 0

					vel_msg.angular.x = 0
					vel_msg.angular.y = 0
					vel_msg.angular.z = self.angular_vel(cir_pose)

					self.velocity_publisher.publish(vel_msg)

					self.rate.sleep()

				if(halfway==1):
					a=a+1
					continue
				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
				self.velocity_publisher.publish(vel_msg)
				a=a+1

			goal_pose.x=self.pose2.x+2*math.cos(theta)
			goal_pose.y=self.pose2.y+2*math.sin(theta)

			while (abs(self.cap(self.pose1.theta) - self.cap(self.steering_angle(goal_pose)))>0.01):
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel(goal_pose)
				print(vel_msg.angular.z)
				self.velocity_publisher.publish(vel_msg)
				self.rate.sleep()
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)

			while self.euclidean_distance(goal_pose, 0) >= 0.001:


				vel_msg.linear.x = self.linear_vel(goal_pose, 0)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel(goal_pose)

				self.velocity_publisher.publish(vel_msg)

				self.rate.sleep()

			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)


			if(ch>0):
				break



if __name__ == '__main__':
	try:
		x = mybot2()
		x.reachborder()
	except rospy.ROSInterruptException:
		pass