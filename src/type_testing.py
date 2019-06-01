#!/usr/bin/env python

'''
ROS Node for testing publishing types in Python
'''

import rospy
from std_msgs.msg import UInt8
import numpy as np


rospy.init_node("type_testing")
test_pub = rospy.Publisher("/type_testing/test", UInt8, queue_size=1)
rate = rospy.Rate(30)

test_value = 0

while not rospy.is_shutdown():
    msg = UInt8()

    msg.data = test_value
    try:
        test_pub.publish(msg)
    except rospy.exceptions.ROSSerializationException:
        test_pub.unregister()
        test_pub = rospy.Publisher("/type_testing/test", UInt8, queue_size=1)
        msg.data = 0
        test_pub.publish(msg)

    test_value += 1
    print("test_value: %d" % test_value)
    if test_value > 300:
        test_value = 0

    rate.sleep()
