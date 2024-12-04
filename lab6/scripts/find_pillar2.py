#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
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
        self.stop = False

    def find_direction(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        r = np.array(self.msg.ranges)
        r[r < 0.01] = 1000000  # 忽略无效数据

        # 获取从最近到最远的距离排序的索引
        sorted_indices = np.argsort(r)
        
        min_distance = None
        min_angle = None

        # 循环查找最近的孤立目标
        for idx in sorted_indices:
            distance = r[idx]
            angle = idx

            # 将角度转换到 -180 到 180 的范围内
            if angle > 180:
                angle -= 360

            # 检查目标两侧是否空旷
            left_empty = r[(idx + 20) % 360] > distance + 0.5
            right_empty = r[(idx - 20) % 360] > distance + 0.5

            if left_empty and right_empty:
                min_distance = distance
                min_angle = angle
                break

        if min_distance is None or min_angle is None:
            rospy.loginfo("No isolated target detected. Waiting for an isolated target...")
            return [None, None]

        # 噪声处理
        if self.last_angle is not None and self.last_distance is not None:
            if abs(self.last_angle - min_angle) > 30 and self.count <= 5:
                min_angle = self.last_angle
                min_distance = self.last_distance
                self.count += 1
                self.stable_count = 0
                print('Count is {}'.format(self.count))
                print('Noise detected!')
            if abs(self.last_angle - min_angle) <= 30:
                self.stable_count += 1
                if self.stable_count > 2:
                    self.count = 0
                    print('Single detection!')

        self.last_angle = min_angle
        self.last_distance = min_distance

        return [min_distance, min_angle]

    def head_to_goal(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and not self.stop:
            [distance, angle] = self.find_direction()
            twist = Twist()

            if distance is not None and angle is not None:
                if (distance > 0.2 or abs(angle) > 2):
                    twist.angular.z = 0.02 * angle
                    twist.linear.x = 0.1 * distance + 0.06
                elif distance > 0.2 and abs(angle) <= 1 and self.stable_count >= 1:
                    twist.angular.z = 0
                    twist.linear.x = 0.2
                    print('Go straight')
                else:
                    twist.angular.z = 0
                    twist.linear.x = 0
                    rospy.loginfo('Reached pole!')
                    self.pub.publish(twist)
                    # 停止2秒
                    time.sleep(2)
                    self.stop = True
                    break
            else:
                # 没有检测到目标，保持静止
                twist.angular.z = 0
                twist.linear.x = 0

            twist.angular.z = np.clip(twist.angular.z, -1.4, 1.4)
            twist.linear.x = np.clip(twist.linear.x, -0.2, 0.2)
            self.pub.publish(twist)
            rospy.loginfo('Angle is {}, Distance is {}'.format(angle, distance if distance is not None else "None"))
            rate.sleep()

def find_pillar():
    # 假设 rospy.init_node 已在主程序中调用
    try:
        detector = Detector()
        detector.head_to_goal()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('detect_pole', anonymous=True)
    find_pillar()

