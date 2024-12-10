#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from movebase import movebase_client
from find_pillar import find_pillar

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

    # 硬编码6个目标位置
    target_positions = [
        (-1.8, 1.0),  # 目标1
        (-2.0, 0.0),  # 目标2
        (1.8, 1.0),   # 目标3
        (2.0, 0.0),    # 目标4
        (-1.8, -1.0),  # 目标5
        (1.8, -1.0)    # 目标6
    ]

    # 每经过两个目标点执行一次find_pillar
    for i in range(0, len(target_positions), 2):
        pair = target_positions[i:i+2]
        for idx, (goal_x, goal_y) in enumerate(pair, start=1):
            global_idx = i + idx  # 全局目标点编号
            rospy.loginfo("State 1: Moving to target position {}: x = {:.2f}, y = {:.2f}".format(global_idx, goal_x, goal_y))
            # 状态1：移动到目标位置
            success = False
            attempt = 0
            max_attempts = 5  # 最大尝试次数，可根据需要调整
            while not success and attempt < max_attempts:
                result = movebase_client(goal_x, goal_y)
                if result:
                    rospy.loginfo("Reached target position {}.".format(global_idx))
                    success = True
                else:
                    attempt += 1
                    rospy.logwarn("Failed to reach target position {}. Attempt {}/{}".format(global_idx, attempt, max_attempts))
                    rospy.loginfo("Retrying to reach target position {}...".format(global_idx))
                    time.sleep(2)  # 等待2秒后重试
            if not success:
                rospy.logerr("Failed to reach target position {} after {} attempts. Exiting.".format(global_idx, max_attempts))
                rospy.signal_shutdown("Unable to reach target position.")
                return

        # 状态2：运行 find_pillar.py
        rospy.loginfo("State 2: Running find_pillar.py to find and approach the pillar.")
        find_pillar()
        rospy.loginfo("Finished approaching pillar. Proceeding to next pair of targets.")

    # 状态3：返回初始位置
    rospy.loginfo("State 3: Returning to initial position.")
    success = False
    attempt = 0
    max_attempts = 5
    while not success and attempt < max_attempts:
        result = movebase_client(init_x, init_y)
        if result:
            rospy.loginfo("Returned to initial position.")
            success = True
        else:
            attempt += 1
            rospy.logwarn("Failed to return to initial position. Attempt {}/{}".format(attempt, max_attempts))
            rospy.loginfo("Retrying to return to initial position...")
            time.sleep(2)  # 等待2秒后重试
    if not success:
        rospy.logerr("Failed to return to initial position after {} attempts.".format(max_attempts))

    rospy.loginfo("State machine execution completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
