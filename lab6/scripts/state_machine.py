#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from movebase import movebase_client
from find_pillar2 import find_pillar

def get_current_pose():
    # 获取机器人当前位姿
    current_pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    current_x = current_pose_msg.pose.pose.position.x
    current_y = current_pose_msg.pose.pose.position.y
    current_orientation = current_pose_msg.pose.pose.orientation
    return current_x, current_y, current_orientation

def main():
    rospy.init_node('state_machine_node')

    # 获取初始位置
    rospy.loginfo("Getting initial position...")
    init_x, init_y, init_orientation = get_current_pose()
    rospy.loginfo("Initial position: x = {:.2f}, y = {:.2f}".format(init_x, init_y))

    # 获取目标位置列表（输入参数）
    import sys
    if len(sys.argv) != 7:
        rospy.loginfo("Usage: state_machine.py x1 y1 x2 y2 x3 y3")
        sys.exit(1)
    target_positions = [
        (float(sys.argv[1]), float(sys.argv[2])),
        (float(sys.argv[3]), float(sys.argv[4])),
        (float(sys.argv[5]), float(sys.argv[6]))
    ]

    for idx, (goal_x, goal_y) in enumerate(target_positions):
        rospy.loginfo("State 1: Moving to target position {}: x = {:.2f}, y = {:.2f}".format(idx+1, goal_x, goal_y))
        # 状态1：移动到目标位置
        result = movebase_client(goal_x, goal_y)
        if result:
            rospy.loginfo("Reached target position {}.".format(idx+1))
        else:
            rospy.logwarn("Failed to reach target position {}.".format(idx+1))
            continue  # 继续下一个目标位置

        # 状态2：运行 find_pillar.py
        rospy.loginfo("State 2: Running find_pillar.py to find and approach the pillar.")
        find_pillar()
        rospy.loginfo("Finished approaching pillar. Waiting for 2 seconds.")
        # 已在 find_pillar.py 中等待了2秒，这里无需再次等待

    # 状态3：返回初始位置
    rospy.loginfo("State 3: Returning to initial position.")
    result = movebase_client(init_x, init_y)
    if result:
        rospy.loginfo("Returned to initial position.")
    else:
        rospy.logwarn("Failed to return to initial position.")

    rospy.loginfo("State machine execution completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

