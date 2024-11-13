#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

def calculate_target_angle(radar_distances, threshold=0.5, min_distance=1, max_distance=1.5):
    distance_gradient = np.diff(radar_distances)
    edges = np.where(np.abs(distance_gradient) > threshold)[0]
    if len(edges) == 0:
        return None, None
    
    pole_angles = []
    pole_distances = []
    for edge in edges:
        if (min_distance <= radar_distances[edge % 360] <= max_distance and
            radar_distances[(edge + 357) % 360] >= min_distance and
            radar_distances[(edge + 3) % 360] >= min_distance):
            pole_angles.append(edge)
            pole_distances.append(radar_distances[edge % 360])

    if len(pole_angles) > 0:
        return pole_distances[0], pole_angles[0]
    else:
        return None, None

def find_direction():
    msg = rospy.wait_for_message("scan", LaserScan)
    r = np.array(msg.ranges)
    r[r < 0.01] = 1000000
    distance, angle = calculate_target_angle(r)
    if angle is not None and angle > 180:
        angle -= 360
    return distance, angle

def head_to_goal():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    last_angle, last_distance = None, None
    count, stable_count = 0, 0

    while not rospy.is_shutdown():
        distance, angle = find_direction()
        twist = Twist()

        if distance is not None and angle is not None:
            if last_angle is not None and abs(last_angle - angle) > 30 and count <= 5:
                angle = last_angle
                distance = last_distance
                count += 1
                stable_count = 0
                print(f'Count is {count}, Noise detected!')
            elif last_angle is not None and abs(last_angle - angle) <= 30:
                stable_count += 1
                if stable_count > 5:
                    count = 0
                    print('Single detection!')

            last_angle = angle
            last_distance = distance

            if (distance > 0.2 or abs(angle) > 2) and angle != 0:
                twist.angular.z = 0.02 * angle
                twist.linear.x = 0.1 * distance + 0.06
            elif distance > 0.2 and abs(angle) <= 1 and stable_count >= 5:
                twist.angular.z = 0
                twist.linear.x = 0.2
                print('Go straight')
            else:
                twist.angular.z = 0
                twist.linear.x = 0
                rospy.loginfo('Reach pole!')

            twist.angular.z = np.clip(twist.angular.z, -1.4, 1.4)
            twist.linear.x = np.clip(twist.linear.x, -0.2, 0.2)
            pub.publish(twist)
            rospy.loginfo(f'Angle is {angle}, Distance is {distance}')
        else:
            rospy.loginfo('No target detected.')

def main():
    rospy.init_node('detect_pole', anonymous=True)
    try:
        head_to_goal()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
