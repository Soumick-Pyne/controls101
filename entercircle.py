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
        
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose, radius):
        return (sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2)) - radius)

    def cap(self, angle):
        if(angle>PI):
            angle=angle-2*PI
        if(angle<(-1*PI)):
            angle=angle+2*PI
        return angle


    def linear_vel(self, goal_pose, radius,constant=1.5):
        return constant * self.euclidean_distance(goal_pose, radius)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2):
        return constant * (self.cap(self.steering_angle(goal_pose) - (self.pose.theta)))

    def reachborder(self):
        goal_pose = Pose()

        goal_pose.x = input("Set your x centre: ")
        goal_pose.y = input("Set your y centre: ")
        radius = input("set your radius: ")
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while (abs(self.cap(self.pose.theta) - self.cap(self.steering_angle(goal_pose)))>0.01):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
         
         

        while self.euclidean_distance(goal_pose, radius) >= distance_tolerance:


            vel_msg.linear.x = self.linear_vel(goal_pose, radius)
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

        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        while abs((self.cap(self.pose.theta) - self.cap(self.steering_angle(goal_pose)))<PI/2):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose) + 3*PI

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        #vel_msg.angular.z = -2*PI
        #vel_msg.linear.x = 2*PI*radius
        #self.velocity_publisher.publish(vel_msg)

        a=1;
        n=2*PI/0.1
        thetanot= self.cap(self.steering_angle(goal_pose))
        
        while(a<n):
            
            halfway = 0
            cir_pose = Pose()
            cir_pose.x = goal_pose.x - radius*math.cos(self.cap(thetanot-a*0.1))
            cir_pose.y = goal_pose.y - radius*math.sin(self.cap(thetanot-a*0.1))
            dis = self.euclidean_distance(cir_pose,0)
            """while (abs(self.pose.theta - self.steering_angle(cir_pose))>0.001):
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(cir_pose)

                self.velocity_publisher.publish(vel_msg)

                self.rate.sleep()

            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)   """     
            while (abs(self.cap(self.steering_angle(cir_pose)) - self.cap(self.pose.theta))>0.01):

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

        
        rospy.spin()

if __name__ == '__main__':
    try:
        x = mybot2()
        x.reachborder()
    except rospy.ROSInterruptException:
        pass