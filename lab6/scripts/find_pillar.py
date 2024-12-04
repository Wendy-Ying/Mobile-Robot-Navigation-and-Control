#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import time

class Detector:
    def __init__(self):
        self.last_distance = None
        self.last_angle = None
        self.count = 0
        self.stable_count = 0
        rospy.loginfo('Detect_pole node initialized')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.head_to_goal()

    def find_direction(self):
        # Wait for a laser scan message
        msg = rospy.wait_for_message("scan", LaserScan)
        r = np.array(msg.ranges)
        r[r < 0.01] = 1000000  # Ignore invalid data, set to a large value

        # Get the indices sorted by distance
        sorted_indices = np.argsort(r)

        min_distance = None
        min_angle = None

        # Find the closest isolated target
        for idx in sorted_indices:
            distance = r[idx]
            angle = idx if idx <= 180 else idx - 360  # Normalize angle to range -180 to 180

            # Check if both sides are clear (to the left and right of the target)
            left_empty = r[(idx + 20) % 360] > distance + 0.5
            right_empty = r[(idx - 20) % 360] > distance + 0.5

            if left_empty and right_empty:
                min_distance = distance
                min_angle = angle
                break  # Found an isolated target

        # If no isolated target found, return None
        if min_distance is None or min_angle is None:
            rospy.loginfo("No isolated target detected. Waiting for an isolated target...")
            return None, None

        # Noise filtering based on the last detected target
        if self.last_angle is not None and self.last_distance is not None:
            angle_diff = abs(self.last_angle - min_angle)
            if angle_diff > 30 and self.count <= 5:
                min_angle = self.last_angle
                min_distance = self.last_distance
                self.count += 1
                self.stable_count = 0
                rospy.loginfo(f"Noise detected! Count is {self.count}")
            elif angle_diff <= 30:
                self.stable_count += 1
                if self.stable_count > 2:
                    self.count = 0
                    rospy.loginfo("Stable detection!")

        # Update the last detected target
        self.last_angle = min_angle
        self.last_distance = min_distance

        return min_distance, min_angle

    def head_to_goal(self):
        while not rospy.is_shutdown():
            distance, angle = self.find_direction()
            twist = Twist()

            if distance is not None and angle is not None:
                # Move towards the target with some conditions
                if distance > 0.2 or abs(angle) > 2:
                    twist.angular.z = 0.02 * angle
                    twist.linear.x = 0.1 * distance + 0.06
                elif distance > 0.2 and abs(angle) <= 1 and self.stable_count >= 1:
                    twist.angular.z = 0
                    twist.linear.x = 0.2
                    rospy.loginfo("Going straight")
                else:
                    twist.angular.z = 0
                    twist.linear.x = 0
                    rospy.loginfo("Reached the pole!")
                    time.sleep(2)
                    talker()  # Trigger the talker once the goal is reached
            else:
                # No target detected, stop the robot
                twist.angular.z = 0
                twist.linear.x = 0

            # Clip the velocities to prevent extreme values
            twist.angular.z = np.clip(twist.angular.z, -1.4, 1.4)
            twist.linear.x = np.clip(twist.linear.x, -0.2, 0.2)
            self.pub.publish(twist)
            rospy.loginfo(f"Angle: {angle}, Distance: {distance if distance is not None else 'None'}")

def talker():
    # Publisher to notify when the robot reaches the goal
    pub = rospy.Publisher('goto', Bool, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        msg = Bool(data=True)
        rospy.loginfo(f"Publishing to 'goto' topic: {msg.data}")
        pub.publish(msg)
        rate.sleep()

def callback(data):
    if data.data:
        rospy.loginfo("Received find_pole: True")
        try:
            detector = Detector()
        except rospy.ROSInterruptException:
            pass

def listener():
    # Subscribe to "find_pole" topic
    rospy.Subscriber("find_pole", Bool, callback)
    rospy.spin()

def main():
    # Initialize the main node only once
    rospy.init_node('detect_pole', anonymous=True)
    listener()

if __name__ == '__main__':
    main()
