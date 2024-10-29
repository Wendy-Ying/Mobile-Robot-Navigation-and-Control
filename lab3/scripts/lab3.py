#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

# store the distance
dis = []
dis_true = 0.464

def callback(scan):
    global dis
    
    dis_min = scan.range_max
    for i in range(0, 360):
        distance = scan.ranges[i]
        if distance < dis_min and distance != 0:
            dis_min = distance
    
    # the minimum distance represent the measured result normal to the wall
    dis.append(dis_min)

def process_data(_):
    global dis
    
    if len(dis) > 0:
        dis_array = np.array(dis)
        rospy.loginfo("accuracy: %f m", abs(dis_array.mean() - dis_true))
        rospy.loginfo("precision: %f m", (dis_array.max() - dis_array.min()) / 2)
        dis.clear()

def listener():
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    
    # range_min and range_max
    scan = rospy.wait_for_message('/scan', LaserScan)
    rospy.loginfo("range min: %f m, max: %f m", scan.range_min, scan.range_max)
    
    # process the data
    rospy.Timer(rospy.Duration(3), process_data)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
