#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('get_robot_pose')
    listener = tf.TransformListener()
    pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)

    # 假设机器人在/base_link坐标系中
    target_frame = '/base_link'
    source_frame = '/map'

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
            # rospy.loginfo("Robot Position: %s", trans)
            # rospy.loginfo("Robot Orientation: %s", rot)

            # 创建一个PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = source_frame
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]

            # 发布消息到话题
            pub.publish(pose_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get robot pose")

        rate.sleep()
