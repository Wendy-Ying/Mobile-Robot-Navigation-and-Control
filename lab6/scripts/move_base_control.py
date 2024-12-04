#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import threading

def movebase_client():
    # 创建一个SimpleActionClient实例，连接到move_base服务
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # 等待move_base服务可用
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")
    
    # 创建一个目标位姿MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # 目标坐标系
    goal.target_pose.header.stamp = rospy.Time.now()  # 当前时间戳

    # 设置目标位置和朝向
    goal.target_pose.pose.position.x = 1.0  # 设置目标位置X轴坐标
    goal.target_pose.pose.position.y = 0.0  # 设置目标位置Y轴坐标
    goal.target_pose.pose.orientation.w = 1.0  # 设置目标朝向（四元数表示）

    # 发送目标给move_base并注册反馈回调
    client.send_goal(goal, feedback_cb=feedback_callback)

    # 等待结果
    client.wait_for_result()

    # 获取执行结果
    result = client.get_result()
    if result:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.loginfo("Failed to reach the goal.")

def cmd_vel_publisher():
    # 发布器，用于控制速度到 /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz发布频率

    # 创建一个Twist消息来发布控制指令
    move_cmd = Twist()
    move_cmd.linear.x = 0.2  # 设置线速度：前进0.2米/秒
    move_cmd.angular.z = 0.0  # 设置角速度：不旋转

    while not rospy.is_shutdown():
        pub.publish(move_cmd)  # 发布速度命令
        rate.sleep()  # 保持频率

def feedback_callback(feedback):
    # 获取move_base的反馈信息（机器人的当前位置）
    rospy.loginfo("Current position: x = %f, y = %f", 
                  feedback.base_position.pose.position.x,
                  feedback.base_position.pose.position.y)

def movebase_action_server():
    # 启动move_base客户端并设置目标
    rospy.init_node('movebase_client')
    
    # 启动控制速度的发布器线程
    cmd_vel_thread = threading.Thread(target=cmd_vel_publisher)
    cmd_vel_thread.start()
    
    # 启动move_base目标设置并等待结果
    movebase_client()

    rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        movebase_action_server()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation task finished.")
