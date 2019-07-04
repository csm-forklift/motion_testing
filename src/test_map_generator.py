#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class TestMap:
    def __init__(self):
        # ROS Objects
        rospy.init_node("test_map_generator")
        self.map_pub = rospy.Publisher("~map", OccupancyGrid, queue_size=1)
        self.target_pub = rospy.Publisher("~target", PoseStamped, queue_size=1)
        self.rate = rospy.Rate(10)
        self.test_map = OccupancyGrid()

        # Target point
        self.target = PoseStamped()
        self.target.header.frame_id = "odom"
        self.target.pose.position.x = 13
        self.target.pose.position.y = 5
        quat_target = quaternion_from_euler(0,0,-0.78539816339)
        self.target.pose.orientation = Quaternion(quat_target[0], quat_target[1], quat_target[2], quat_target[3])

        # Create Map
        self.test_map.header.frame_id = "odom"
        self.test_map.info.width = 40
        self.test_map.info.height = 40
        self.test_map.info.resolution = 0.50
        self.test_map.info.origin.position.x = -2
        self.test_map.info.origin.position.y = -10# Create obstacles

        # Add obstacles
        self.test_map.data = [0] * (self.test_map.info.width*self.test_map.info.height)
        for i in range(20, 31):
            for j in range(20, 26):
                self.test_map.data[self.rowMajorTo1D(i, j, self.test_map.info.width)] = 100

        for i in range(4,9):
            for j in range(32,37):
                self.test_map.data[self.rowMajorTo1D(i, j, self.test_map.info.width)] = 100

    def spin(self):
        while not rospy.is_shutdown():
            self.test_map.header.stamp = rospy.Time.now()
            self.map_pub.publish(self.test_map)
            self.target_pub.publish(self.target)
            self.rate.sleep()

    def rowMajorTo1D(self, row, col, width):
        '''
        Converts 2D matrix indices to a row-major 1D vector. Starts at (0,0).
        '''
        i = row*width + col

        return i

if __name__ == "__main__":
    try:
        test_map = TestMap()
        test_map.spin()
    except rospy.ROSInterruptException:
        pass
