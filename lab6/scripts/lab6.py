#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time


# MoveBase Navigation
def movebase_client(x, y, w, z):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

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


# Pole Detection Logic
class Detector:
    def __init__(self):
        self.last_distance = None
        self.last_angle = None
        self.count = 0
        rospy.loginfo('Pole detection started')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def find_direction(self):
        msg = rospy.wait_for_message("scan", LaserScan)
        r = np.array(msg.ranges)
        r[r < 0.01] = 1000000  # Ignore invalid data, set to a large value

        sorted_indices = np.argsort(r)
        for idx in sorted_indices:
            distance = r[idx]
            angle = idx if idx <= 180 else idx - 360
            left_empty = r[(idx + 20) % 360] > distance + 0.5
            right_empty = r[(idx - 20) % 360] > distance + 0.5

            if left_empty and right_empty:
                return distance, angle

        return None, None

    def head_to_goal(self):
        while not rospy.is_shutdown():
            distance, angle = self.find_direction()
            twist = Twist()

            if distance and angle:
                if distance > 0.15 and abs(angle) > 2:
                    twist.angular.z = 0.05 * angle
                    twist.linear.x = 0.1 * distance + 0.06
                    rospy.loginfo("Changing direction")
                elif distance > 0.15 and abs(angle) <= 1:
                    twist.angular.z = 0
                    twist.linear.x = 0.22
                    rospy.loginfo("Going straight")
                else:
                    twist.angular.z = 0
                    twist.linear.x = 0
                    rospy.loginfo("Reached the pole!")
                    time.sleep(2)
                    twist.linear.x = -0.22
                    self.pub.publish(twist)
                    time.sleep(2)
                    return  # Detection complete, exit

            twist.angular.z = np.clip(twist.angular.z, -1.4, 1.4)
            twist.linear.x = np.clip(twist.linear.x, -0.22, 0.22)
            self.pub.publish(twist)
            # rospy.loginfo(f"Angle: {angle}, Distance: {distance if distance is not None else 'None'}")


# Navigation sequence with via points and goal points
def global_planner():
    waypoints = [
        ("via", [(-4.39, -1.22, 0.97, -0.20)]),
        ("goal", [(-3.09, -1.65, 0.98, -0.14)]),
        ("via", [(-2.14, 0.70, 0.04, 0.33)]),
        ("goal", [(-0.82, 0.34, 0.55, -0.83)]),
        ("goal", [(-2.58, 1.15, 0.19, 0.98)]),
        ("goal", [(-5.32, -1.30, 1.0, 0.0)])
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
