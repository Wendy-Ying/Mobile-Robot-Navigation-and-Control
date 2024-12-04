#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool

# This function will send the move goal with provided parameters
def movebase_client(x, y, w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

# Publisher to notify when the robot reaches the goal
def talker():
    rospy.init_node('find_pole_talker', anonymous=True)
    pub = rospy.Publisher('find_pole', Bool, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        msg = Bool(data=True)
        rospy.loginfo(f"Publishing to 'find_pole' topic: {msg.data}")
        pub.publish(msg)
        rate.sleep()

# Callback function for the 'goto' topic
def goto_callback(msg):
    global goto_counter

    if msg.data:  # If True, change the target position
        if goto_counter == 0:
            rospy.loginfo("Moving to first position (-1.1, 0.4, 1.0)")
            movebase_client(-1.1, 0.4, 1.0)
        elif goto_counter == 1:
            rospy.loginfo("Moving to second position (1, 1, 1)")
            movebase_client(1, 1, 1)
        elif goto_counter == 2:
            rospy.loginfo("Moving to third position (2, 2, 0.5)")
            movebase_client(2, 2, 0.5)

        goto_counter += 1

def movebase_node():
    global goto_counter
    goto_counter = 0  # Initialize the counter to control the sequence of movements

    # Initialize the ROS node
    rospy.init_node('movebase_client_py')

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
