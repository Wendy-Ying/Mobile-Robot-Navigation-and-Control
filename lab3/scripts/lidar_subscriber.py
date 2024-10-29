#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(scan):
    rospy.loginfo("range min: %f, max: %f",scan.range_min, scan.range_max)

    # 读取特定角度的数据
    index = 0
    if index < len(scan.ranges):
        distance = scan.ranges[index]
        intensity = scan.intensities[index] if len(scan.intensities) > index else None

        rospy.loginfo("Distance: %f, Intensity: %f", distance, intensity if intensity is not None else 0.0)
    else:
        rospy.logwarn("Index out of range")

def listener():
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
