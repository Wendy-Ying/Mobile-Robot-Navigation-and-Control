#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def callback(scan):
    distance_list = []
    for index in range(1,len(scan.ranges)):
        distance = scan.ranges[index]
        distance_list.append(distance)
    angle = calculate_target_angle(distance_list)
    if len(angle) > 0:
        print(angle[0])
        

def calculate_target_angle(radar_distances, threshold=0.5, min_distance=1, max_distance=1.5):
    
    # compute gradient
    distance_gradient = np.diff(radar_distances)
    # find edge
    edges = np.where(np.abs(distance_gradient) > threshold)[0]
    if len(edges) == 0:
        return None
    
    # limit the distance
    pole_angle = []
    for edge in edges:
        if (min_distance <= radar_distances[edge % 360] <= max_distance and
            radar_distances[(edge + 357) % 360] >= min_distance and
            radar_distances[(edge + 3) % 360] >= min_distance):
            pole_angle.append(edge)
    return pole_angle
    

def listener():
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
