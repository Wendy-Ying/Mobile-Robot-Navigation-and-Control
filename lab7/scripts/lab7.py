#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import math
import tf

# MoveBase Navigation
def movebase_client(x, y, z=0, w=0):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    if z == 0 or w == 0:
        z, w = calculate_orientation(x, y)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.pose.orientation.z = z

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return False
    else:
        return True

def calculate_orientation(x, y):
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(5.0))
    (trans, _) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
    current_x, current_y = trans[0], trans[1]
    dx = x - current_x
    dy = y - current_y
    yaw = math.atan2(dy, dx)
    from tf.transformations import quaternion_from_euler
    quat = quaternion_from_euler(0, 0, yaw)
    return quat[2], quat[3]

# Pole Detection Logic
class Detector:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.loginfo('Pole detection started')

    def find_direction(self):
        msg = rospy.wait_for_message("scan", LaserScan)
        r = np.array(msg.ranges)
        r[r < 0.05] = 1000000  # Ignore invalid data, set to a large value

        sorted_indices = np.argsort(r)
        for idx in sorted_indices:
            distance = r[idx]
            angle = idx if idx <= 180 else idx - 360
            left_empty = r[(idx + 20) % 360] > distance + 0.3
            right_empty = r[(idx - 20) % 360] > distance + 0.3

            if left_empty and right_empty:
                return distance, angle

        return None, None

    def head_to_goal(self):
        consecutive_reaches = 0  # Counter for consecutive "reach" detections
        while not rospy.is_shutdown():
            distance, angle = self.find_direction()
            twist = Twist()

            if distance and angle:
                rospy.loginfo(f"Distance: {distance}, Angle: {angle}")
                if distance < 0.4:
                    consecutive_reaches += 1
                    twist.linear.x = 0.22
                    self.pub.publish(twist)
                    time.sleep(1)
                    rospy.loginfo(f"Consecutive reaches: {consecutive_reaches}")
                else:
                    consecutive_reaches = 0  # Reset counter if no valid detection

                if consecutive_reaches >= 2:  # Check for 2 consecutive readings
                        rospy.loginfo("Reached the pole!")
                        twist.angular.z = 0
                        twist.linear.x = 0
                        self.pub.publish(twist)
                        time.sleep(2)
                        twist.linear.x = -0.22
                        self.pub.publish(twist)
                        time.sleep(2)
                        return  # Detection complete, exit
                else:
                    if abs(angle) < 10:
                        twist.angular.z = 0.01 * angle
                        twist.linear.x = 0.22
                        rospy.loginfo("Going straight")
                        self.pub.publish(twist)
                    else:
                        twist.linear.x = 0.22
                        twist.angular.z = 0.1 * angle
                        rospy.loginfo("Turning")
                        self.pub.publish(twist)
            else:
                consecutive_reaches = 0  # Reset counter if no valid detection

            twist.angular.z = np.clip(twist.angular.z, -1.4, 1.4)
            twist.linear.x = np.clip(twist.linear.x, -0.22, 0.22)
            self.pub.publish(twist)


# Navigation sequence with via points and goal points
def global_planner():
    # Get the current position of the robot
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(5.0))
    (trans, _) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
    current_x, current_y = trans[0], trans[1]

    waypoints = [
        ("via", [(-4.73, -1.15, -0.34, 0.93)]),
        ("goal", [(-3.09, -1.65)]),
        ("via", [(-2.14, 0.70)]),
        ("goal", [(-0.42, 0.06)]),
        ("goal", [(-2.94, 1.57)]),
        ("via", [(-2.19, 0.66)]),
        ("via", [(-3.80, -1.68, 0.97, 0.23)]),
        ("via", [(current_x, current_y)])
    ]

    for idx, (wp_type, points) in enumerate(waypoints):
        rospy.loginfo(f"Executing {wp_type} points for waypoint {idx + 1}")

        for wp in points:
            result = movebase_client(*wp)
            if not result:
                rospy.logerr(f"Failed to reach point {wp}")
                return

        if wp_type == "goal":
            rospy.loginfo(f"Reached goal waypoint {idx + 1}, starting pole detection.")
            Detector().head_to_goal()


# Main function
def main():
    rospy.init_node('navigation_with_detection', anonymous=True)
    global_planner()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation and pole detection complete.")
