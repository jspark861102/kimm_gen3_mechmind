#!/usr/bin/env python3
import time
from math import pi
import numpy as np

import rospy
from tcp_ros.msg import Trigger, GetResult, Feedback
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from std_msgs.msg import String
from tf.transformations import *

def tf_to_pose_msg(mat):
    t = mat[:3, 3]
    q = quaternion_from_matrix(mat)

    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]

    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    return pose_msg

def tf_from_pose_msg(pose_msg):
    q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    t = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    T_mat = quaternion_matrix(q)
    T_mat[:3, 3] = t

    return T_mat

def tf_from_pose(translation, quaternion):
    T_mat = quaternion_matrix(quaternion)
    T_mat[:3, 3] = translation

    return T_mat

def pub_trigger_msg(pub_trigger):   
    msg = Trigger()
    msg.project = '1'
    msg.request = '20'
    
    pub_trigger.publish(msg)
    
    print('published trigger')
    
def pub_get_result_msg(pub_getresult):
    msg = GetResult()
    msg.project = '1'
    
    pub_getresult.publish(msg)    
    
    print('published getResult msg')
    
def feedback_msg_callback(data):
    print('get pose data from the mechmind')
    # print(len(data.position))
    # print(data.position[0])
    
    pose_array_msgs = PoseArray()
    pose_array_msgs.header = data.header

    marker_pose = Pose()
    marker_pose.position.x = 0.18189094960689545
    marker_pose.position.y = 0.033538348972797394
    marker_pose.position.z = 1.0663881301879883
    marker_pose.orientation.x = 0.7163586426758909
    marker_pose.orientation.y = 0.6958243073782278
    marker_pose.orientation.z = -0.02424269392995153
    marker_pose.orientation.w = -0.04550973015139544
    pose_array_msgs.poses.append(marker_pose)

    T_cm = tf_from_pose_msg(marker_pose)
    T_mc = inverse_matrix(T_cm)
    
    for pose in data.position:
        #print(pose.label)
        #print(pose.linear.x*1e-3, pose.linear.y*1e-3, pose.linear.z*1e-3)
        #print(pose.angular.x*pi/180.0, pose.angular.y*pi/180.0, pose.angular.z*pi/180.0)
        
        t_obj = (pose.linear.x*1e-3, pose.linear.y*1e-3, pose.linear.z*1e-3)
        q_obj = quaternion_from_euler(pose.angular.x*pi/180.0, pose.angular.y*pi/180.0, pose.angular.z*pi/180.0)
        
        pose_msg = Pose()
        pose_msg.position.x = pose.linear.x*1e-3
        pose_msg.position.y = pose.linear.y*1e-3
        pose_msg.position.z = pose.linear.z*1e-3

        # print(q_obj)
        pose_msg.orientation = Quaternion(q_obj[0], q_obj[1], q_obj[2], q_obj[3])
        
        pose_array_msgs.poses.append(pose_msg)
        
    # print(pose_array_msgs)
    if len(pose_array_msgs.poses) > 2:
        T_co = tf_from_pose_msg(pose_array_msgs.poses[1])
        T_mo = np.matmul(T_mc, T_co)

        # print(T_mo)

        pub_msg = tf_to_pose_msg(T_mo)

        pub_obj_pose.publish(pub_msg)
        # print(pub_msg)
    
    pub_obj_poses.publish(pose_array_msgs)

def call_from_robot(msg):
    time.sleep(1)
    pub_trigger_msg(pub_trigger)

    time.sleep(1)
    pub_get_result_msg(pub_getresult)

    time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('object_pose_node')#, anonymous=True)
    pub_trigger = rospy.Publisher('/trigger', Trigger, queue_size=10)
    pub_getresult = rospy.Publisher('/getResult', GetResult, queue_size=10)
    
    pub_obj_poses = rospy.Publisher('/obj_poses', PoseArray, queue_size=10)

    pub_obj_pose = rospy.Publisher('mechmind_publisher/pose', Pose, queue_size=10)

    rospy.Subscriber('kimm_gen3_pnp/mech_call', String, call_from_robot)
    rospy.Subscriber('/pose', Feedback, feedback_msg_callback)

    rospy.spin()


    
# rostopic pub /trigger tcp_ros/Trigger "project: '1'"
# rostopic pub /trigger tcp_ros/Trigger "request: '20'"
# rostopic pub /getResult tcp_ros/GetResult "project: '1'"
