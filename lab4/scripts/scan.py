#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def callback(scan):
    distance_list = []
    for index in range(0,len(scan.ranges)):
        distance = scan.ranges[index]
        distance_list.append(distance)
    dis, angle = calculate_target_angle(distance_list)
    print(dis, angle)
        

def calculate_target_angle(radar_distances, threshold=0.5):
    distance_gradient = np.diff(radar_distances)
    edges = np.where(np.abs(distance_gradient) > threshold)[0]
    if len(edges) == 0:
        return None, None
    
    pole_angles = []
    pole_distances = []

    # 定义偏移量和距离差，用于简化循环逻辑
    offset_distance_pairs = [
        (0, 0.5),
        (1, 0.5),
        (-1, 0.5)
    ]

    for edge in edges:
        for offset, distance_diff in offset_distance_pairs:
            dis1 = radar_distances[(edge + 3 + offset) % 360]
            dis2 = radar_distances[(edge + offset) % 360]
            dis3 = radar_distances[(edge + 357 + offset) % 360]
            
            # 检查条件并存储满足条件的角度和距离
            if (dis1 - dis2 > distance_diff) and (dis3 - dis2 > distance_diff) and dis2 > 0:
                pole_angles.append((edge + offset) % 360)
                pole_distances.append(dis2)
                break  # 一旦找到满足条件的就跳出内层循环

    if pole_angles:
        return pole_distances, pole_angles
    return None, None
    

def listener():
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
