#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

class TestMap:
    def __init__(self):
        # ROS Objects
        rospy.init_node("test_map_generator")
        self.map_pub = rospy.Publisher("~map", OccupancyGrid, queue_size=1)
        self.rate = rospy.Rate(10)
        self.test_map = OccupancyGrid()

        # Create Map
        self.test_map.header.frame_id = "odom"
        self.test_map.info.width = 10
        self.test_map.info.height = 10
        self.test_map.info.resolution = 0.5
        self.test_map.data = [0] * (self.test_map.info.width*self.test_map.info.height)
        self.test_map.data[0] = 100
        self.test_map.data[19] = 100

    def spin(self):
        while not rospy.is_shutdown():
            self.test_map.header.stamp = rospy.Time.now()
            self.map_pub.publish(self.test_map)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        test_map = TestMap()
        test_map.spin()
    except rospy.ROSInterruptException:
        pass
