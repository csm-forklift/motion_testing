#!/usr/bin/env python
'''
Generates a ROS nav_msgs/Odometry message for use with testing the path tracking
controller to make sure the velocity and steering angle setpoints are
reasonable.
'''

import rospy
from nav_msgs.msg import Odometry


class TestOdom:
    def __init__(self):
        #===== Initialize ROS Objects =====#
        rospy.init_node("test_odom_generator")
        self.odom_pub = rospy.Publisher("~test_odom", Odometry, queue_size=1)
        self.rate = rospy.Rate(30)
        self.test_odom = Odometry()
        self.test_odom.header.frame_id = "odom"

        #===== Define Test Odom Pose =====#
        pose_x = .5
        pose_y = .1
        pose_z = 0
        orient_x = 0
        orient_y = 0
        orient_z = 0.0436194
        orient_w = 0.9990482

        #===== Convert into ROS Path Message =====#
        self.test_odom.pose.pose.position.x = pose_x
        self.test_odom.pose.pose.position.y = pose_y
        self.test_odom.pose.pose.position.z = pose_z
        self.test_odom.pose.pose.orientation.x = orient_x
        self.test_odom.pose.pose.orientation.y = orient_y
        self.test_odom.pose.pose.orientation.z = orient_z
        self.test_odom.pose.pose.orientation.w = orient_w

    def spin(self):
        while not rospy.is_shutdown():
            self.test_odom.header.seq += 1
            self.test_odom.header.stamp = rospy.Time.now()
            self.odom_pub.publish(self.test_odom)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        test_odom = TestOdom()
        test_odom.spin()
    except rospy.ROSInterruptException:
        pass
