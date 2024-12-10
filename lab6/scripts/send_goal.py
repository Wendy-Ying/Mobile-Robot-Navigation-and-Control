#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool

# This function will send the move goal with provided parameters
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
    else:
        return client.get_result()

# Publisher to notify when the robot reaches the goal
def talker():
    pub = rospy.Publisher('find_pole', Bool, queue_size=10)
    rospy.loginfo(f"Publishing to 'find_pole' topic: True")
    msg = Bool(data=True)
    pub.publish(msg)
    rospy.sleep(1)
    pub.publish(msg)
    rospy.sleep(1)
    pub.publish(msg)
    rospy.sleep(1)
    msg = Bool(data=False)
    pub.publish(msg)
    rospy.sleep(1)
    pub.publish(msg)
    rospy.sleep(1)

def golobal_planner():
    global goto_counter

    if goto_counter == 0:
        rospy.loginfo("Moving to first pole start point")
        # movebase_client(-4.39, -1.22, 0.97, -0.20)
        result = movebase_client(-3.09, -1.65, 0.98, -0.14)
        if result:
            talker()
            rospy.sleep(1)
            return
    elif goto_counter == 1:
        rospy.loginfo("Moving to second pole start point")
        movebase_client(-0.51, -0.25, 0.78, -0.68)
        result = movebase_client(-1.11, 0.66, 0.78, -0.61)
        if result:
            talker()
            rospy.sleep(1)
            return
    elif goto_counter == 2:
        rospy.loginfo("Moving to third pole start point")
        movebase_client(-1.95, 0.99, 0.21, 0.91)
        result = movebase_client(-3.31, 1.66, 1.0, 0.0)
        if result:
            talker()
            rospy.sleep(1)
            return
    elif goto_counter == 3:
        rospy.loginfo("Moving to fourth pole start point")
        result = movebase_client(-5.32, -1.30, 1.0, 0.0)
        if result:
            return

def goto_callback(msg):
    global goto_counter
    if msg.data:
        goto_counter += 1
        golobal_planner()

def movebase_node():
    global goto_counter
    goto_counter = 0  # Initialize the counter to control the sequence of movements

    # Initialize the ROS node
    rospy.init_node('movebase_client')

    golobal_planner()
    # Subscribe to the "goto" topic
    rospy.Subscriber('goto', Bool, goto_callback)

    # Start the ROS loop
    rospy.spin()

if __name__ == '__main__':
    try:
        # Call the movebase_node to handle the movement logic
        movebase_node()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
