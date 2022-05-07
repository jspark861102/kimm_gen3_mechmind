#!/usr/bin/env python
import sys
import rospy
import tf

import geometry_msgs.msg 

def myhook():
    print("shutdown time!")

if __name__ == '__main__': 
##### ROS #####
    rospy.init_node('vision_test', anonymous=True)
    pub = rospy.Publisher('/my_kinova/vision_pose', geometry_msgs.msg.Pose, queue_size=10)
    rospy.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        roll = 30.3 * 3.141592/180.0
        pitch = -120.1 * 3.141592/180.0
        yaw = 6.7 * 3.141592/180.0

        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw, 'rxyz')
        print(q)

        vision_pose = geometry_msgs.msg.Pose()
        vision_pose.position.x = 0.50
        vision_pose.position.y = 0.05
        vision_pose.position.z = 0.1
        vision_pose.orientation.x = q[0]
        vision_pose.orientation.y = q[1]
        vision_pose.orientation.z = q[2]
        vision_pose.orientation.w = q[3]

        pub.publish(vision_pose)
        # rospy.sleep(1)

        # rate.sleep()
        # rospy.signal_shutdown(myhook)
        rospy.spin()