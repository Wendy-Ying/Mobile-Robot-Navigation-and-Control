#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class vel_manipulator:
    def __init__(self):
        pub_topic_name = "/turtle1/cmd_vel"
        sub_topic_name = "/turtle1/pose"

        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Pose, self.pose_callback)
        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 3
        self.pub.publish(self.velocity_msg)
        self.state = "moving"

    def pose_callback(self, pose):
        rospy.loginfo("%f,%f,%f",pose.x, pose.y, pose.theta)

        if self.state == "moving" and pose.x > 10:
            self.velocity_msg.linear.x = 0
            self.turn(math.pi - 2 * pose.theta)
            self.state = "turning"
        elif self.state == "moving" and pose.x < 1:
            self.velocity_msg.linear.x = 0
            self.turn(3 * math.pi - 2 * pose.theta)
            self.state = "turning"
        elif self.state == "moving" and pose.y > 10:
            self.velocity_msg.linear.x = 0
            self.turn(4 * math.pi - 2 * pose.theta)
            self.state = "turning"
        elif self.state == "moving" and pose.y < 1:
            self.velocity_msg.linear.x = 0
            self.turn(2 * math.pi - 2 * pose.theta)
            self.state = "turning"

        elif self.state == "turning":
            self.velocity_msg.linear.x = 3
            self.velocity_msg.angular.z = 0
            if pose.x <= 10 and pose.x >= 1 and pose.y <= 10 and pose.y >= 1:
                self.state = "moving"

        self.pub.publish(self.velocity_msg)
    
    def turn(self, angle):
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        rate = rospy.Rate(10)
        angular_speed = math.pi / 4
        turn_duration = abs(angle) / angular_speed

        self.velocity_msg.angular.z = angular_speed if angle > 0 else -angular_speed
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < turn_duration:
            self.pub.publish(self.velocity_msg)
            rate.sleep()

        self.velocity_msg.angular.z = 0
        self.pub.publish(self.velocity_msg)


if __name__ == '__main__':
    node_name = "Turtlesim_Saver"
    rospy.init_node(node_name)
    vel_manipulator()
    rospy.spin()