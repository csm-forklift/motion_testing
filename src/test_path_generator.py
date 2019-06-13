#!/usr/bin/env python
'''
Generates a ROS path to be used for testing the velocity controller node
along with the localization.
'''

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time


class TestPath:
    def __init__(self):
        #===== Initialize ROS Objects =====#
        rospy.init_node("test_path_generator")
        self.path_pub = rospy.Publisher("~test_path", Path, queue_size=1)
        self.rate = rospy.Rate(1)
        self.test_path = Path()
        self.test_path.header.frame_id = "odom"
        self.start_time = time.time()

        #===== Define Test Path Points =====#
        # 3m right turn
        points = [[0,0],
                  [3,0],
                  [3,-3]]

        # # Straight line
        # points = [[0,0],
        #           [0.5,0],
        #           [1.0,0],
        #           [1.5,0],
        #           [2.0,0],
        #           [2.5,0],
        #           [3.0,0]]

        #===== Convert into ROS Path Message =====#
        for i in range(len(points)):
            pose = PoseStamped()
            pose.pose.position.x = points[i][0]
            pose.pose.position.y = points[i][1]
            pose.pose.orientation.w = 1
            self.test_path.poses.append(pose)

    def spin(self):
        while not rospy.is_shutdown():
            if ((time.time() - self.start_time) < 6.0):
                self.test_path.header.seq += 1
                self.test_path.header.stamp = rospy.Time.now()
                self.path_pub.publish(self.test_path)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        test_path = TestPath()
        test_path.spin()
    except rospy.ROSInterruptException:
        pass
