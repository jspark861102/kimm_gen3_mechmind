#!/usr/bin/env python3
import time
from math import pi
import numpy as np

import rospy
from sensor_msgs.msg import JointState

def joint_publisher():
    rospy.init_node('joint_state_publisher_ex')
    joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    joint_msg = JointState()
    joint_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'finger_joint', 'left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_outer_knuckle_joint', 'right_inner_knuckle_joint','right_inner_finger_joint']    

    freq = 0.5;
    rate = rospy.Rate(50) 
    while not rospy.is_shutdown():
        mag = 1*np.sin(2.0*pi*freq*rospy.get_time())        
        
        joint_msg.position= [mag, 1.0, 0.0, -1.0, 0.0, -mag, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_msg.header.stamp = rospy.get_rostime()
        joint_states_pub.publish(joint_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass




