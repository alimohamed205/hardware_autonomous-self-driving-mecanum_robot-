#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose():
    rospy.init_node('set_initial_pose')

    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Give some time to connect

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    initial_pose.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                    0, 0.25, 0, 0, 0, 0,
                                    0, 0, 0.25, 0, 0, 0,
                                    0, 0, 0, 0.06853891945200942, 0, 0,
                                    0, 0, 0, 0, 0.06853891945200942, 0,
                                    0, 0, 0, 0, 0, 0.06853891945200942]

    initial_pose_pub.publish(initial_pose)
    rospy.loginfo("Initial pose set")

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass
