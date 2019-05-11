#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, sqrt, atan2,cos,sin,exp
import math
from numpy import ndarray

PI =  3.1415926535897
gamma = 0.3
ang = PI/60

class mybot2:

	def __init__(self):

		rospy.init_node('turtlebot_controller',anonymous = True)

		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size = 10)
		self.velocity_publisher2 = rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size = 10)
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

	def euclidean_distance(self, goal_pose1,self_pose1, radius=0):
		return (sqrt(pow((goal_pose1.x - self_pose1.x), 2) + pow((goal_pose1.y - self_pose1.y), 2)) - radius)

	def linear_vel(self, goal_pose2, radius=0,constant=2):
		print(goal_pose2.x," ",goal_pose2.y)
		print(self.pose.x," ",self.pose.y," ",self.pose.theta)
		print("\n")
		return constant * self.euclidean_distance(goal_pose2,self.pose)

	def cap(self, angle):
		if(angle>PI):
			angle=angle-2*PI
		if(angle<(-1*PI)):
			angle=angle+2*PI
		return angle

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose1.y, goal_pose.x - self.pose1.x)

	def angular_vel(self, goal_pose, constant=2):
		return constant * (self.cap(self.steering_angle(goal_pose) - (self.pose1.theta)))
		
	def goal_potential(self, goal_pose,self_pose,wa=-60):
		return wa*pow(self.euclidean_distance(goal_pose,self_pose,0),-2)

	def obstacle_potential(self, obstacle_pose,self_pose, R, r, wb=1):
		return wb*exp(-1*(pow(self.euclidean_distance(self_pose,obstacle_pose,0),2)-R*R-r*r))

	def evade(self):

		index = 0
		m=0
		t=0

		poses = ndarray((1000,),float)

		vel_msg = Twist()
		vel_msg1 = Twist()

		self_pose = Pose()
		self_pose_old = Pose()

		goal_pose = Pose()
		goal_pose.x = input("Set your x goal:")
		goal_pose.y = input("Set your y goal: ")
		self_pose_old.x = self.pose.x
		self_pose_old.y = self.pose.y
		self_pose_old.theta = self.pose.theta

		print(self.pose)

		while(self.euclidean_distance(goal_pose,self_pose)>0.2):
			k=0
			if(k==0):
				a=0
				b=120
				n=a

			else:
				a=-40
				b=40
				n=a

			while(1):
				self_pose.x = self_pose_old.x + gamma*cos(self_pose_old.theta+n*ang)
				self_pose.y = self_pose_old.y + gamma*sin(self_pose_old.theta+n*ang)

				if(n==a):
					desangle = self.cap(self_pose_old.theta)
					Ut = self.goal_potential(goal_pose,self_pose) + max(self.obstacle_potential(self.pose2,self_pose,0.05,0.05),self.obstacle_potential(self.pose3,self_pose,0.05,0.05))
					Umin = Ut
				else:
					Ut = self.goal_potential(goal_pose,self_pose) + max(self.obstacle_potential(self.pose2,self_pose,0.05,0.05),self.obstacle_potential(self.pose3,self_pose,0.05,0.05))
					if(Ut<=Umin):
						Umin = Ut
						desangle = self.cap(self_pose_old.theta + n*ang)
				n=n+1

				if(n >= b):
					break
			
			self_pose_old.x = self_pose_old.x + gamma*cos(desangle)
			self_pose_old.y = self_pose_old.y + gamma*sin(desangle)
			self_pose_old.theta = desangle

			poses[index] = self_pose_old.x
			index = index + 1
			poses[index] = self_pose_old.y
			index = index + 1
			poses[index] = desangle
			index = index +1

			print("\n")
			print(poses[index-3]," ",poses[index-2]," ",poses[index-1])

			k =k+1

			##############################################################################
		p=0
		print("out of loop 1")
		print("index",index)


		while(p<=(index-3)):

			self_pose.x = poses[p]
			self_pose.y = poses[p+1]
			self_pose.theta = poses[p+2]
			p = p+3

			while (abs(self.cap(self.pose.theta-self_pose.theta))>0.01):
				print("\n")
				if(m>0):
					break
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 2*(self.cap(self_pose.theta-self.pose.theta))
				self.velocity_publisher.publish(vel_msg)
				self.rate.sleep()
			vel_msg.angular.z = 0
			self.velocity_publisher.publish(vel_msg)

			print("out of angle pid")
			
			while self.euclidean_distance(self_pose, self.pose) >0.01:

				if(self.euclidean_distance(self_pose, self.pose)<=0.15):
					m=m+1
					break
				vel_msg.linear.x = self.linear_vel(self_pose)
				#print(vel_msg.linear.x)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				if(m>0):
					vel_msg.angular.z = 2*(self.cap(self_pose.theta-self.pose.theta))
				else:
					vel_msg.angular.z = 0

				self.velocity_publisher.publish(vel_msg)

				self.rate.sleep()

			if(m==0):
				vel_msg.linear.x = 0
				self.velocity_publisher.publish(vel_msg)



		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
			
		rospy.spin()

if __name__ == '__main__':
	try:
		x = mybot2()
		x.evade()
	except rospy.ROSInterruptException:
		pass

			
