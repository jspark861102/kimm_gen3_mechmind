#!/usr/bin/env python3
import time
from math import pi

import rospy
from tcp_ros.msg import Trigger, GetResult, Feedback
from geometry_msgs.msg import PoseArray, Pose, Quaternion
import numpy as np

import tf
from tf.transformations import *

def get_marker_msg():
    marker_pose1 = Pose()
    marker_pose1.position.x = 0.07974227517843246
    marker_pose1.position.y = 0.06610012799501419
    marker_pose1.position.z = 0.2541747987270355
    marker_pose1.orientation.x = 0.6229449356178236
    marker_pose1.orientation.y = -0.5063806891029878
    marker_pose1.orientation.z = 0.38682005622127724
    marker_pose1.orientation.w = 0.4537493096575476

    marker_pose2 = Pose()
    marker_pose2.position.x = -0.1096542552113533
    marker_pose2.position.y = 0.06198103353381157
    marker_pose2.position.z = 0.25281432271003723
    marker_pose2.orientation.x = -0.48643614801166873
    marker_pose2.orientation.y = 0.6349685828653038
    marker_pose2.orientation.z = -0.48347708859144445
    marker_pose2.orientation.w = -0.3555905960538234

    return (marker_pose1, marker_pose2)

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

def inverse_tf(mat):
    invR = mat[:3, :3].transpose()
    t = mat[:3, 3]

    inv_t = np.matmul(-invR,t)

    inv_mat = mat
    inv_mat[:3, :3] = invR

    inv_mat[:3, 3] = inv_t

    return inv_mat

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

if __name__ == '__main__':
    rospy.init_node('marker_pose_node')#, anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    # marker26_msg, marker36_msg = get_marker_msg()
    #
    # T_m26 = tf_from_pose_msg(marker26_msg)
    # T_m36 = tf_from_pose_msg(marker36_msg)

    # inverse_tf(T_m26)

    while not rospy.is_shutdown():
        try:
            # trans = [x, y, z] rot = [x, y, z, w]
            (trans1, quat1) = listener.lookupTransform('/camera_0', '/marker26', rospy.Time(0))
            (trans2, quat2) = listener.lookupTransform('/camera_0', '/marker36', rospy.Time(0))
            (trans_rel, quat_rel) = listener.lookupTransform('/marker26', '/marker36', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        T_cam_26 = tf_from_pose(trans1, quat1)
        T_cam_36 = tf_from_pose(trans2, quat2)
        T_26_36 = tf_from_pose(trans_rel, quat_rel)

        m26_msg = tf_to_pose_msg(T_cam_26)
        m36_msg = tf_to_pose_msg(T_cam_36)

        T_m26 = tf_from_pose_msg(m26_msg)
        T_m36 = tf_from_pose_msg(m36_msg)

        # print(T_26_36)

        q = quaternion_from_matrix(T_26_36)

        # print('T_m26')
        # print(T_m26)
        # print(inverse_tf(T_m26))
        # print('T_m36')
        # print(T_m36)
        print(np.matmul(inverse_matrix(T_m26), T_m36))
        print()
        print('T_26_36')
        print(T_26_36)




# rostopic pub /trigger tcp_ros/Trigger "project: '1'"
# rostopic pub /trigger tcp_ros/Trigger "request: '20'"
# rostopic pub /getResult tcp_ros/GetResult "project: '1'"

