#!/usr/bin/env python
'''
Publishes a constant gear value. To be used when debugging and are not running
the gear chaning nodes.
'''

import rospy
from std_msgs.msg import Int8

def publish_gear():
    rospy.init_node("constant_gear_publisher")
    gear_pub = rospy.Publisher("~gear", Int8, queue_size=3)
    gear = Int8()
    rate = rospy.Rate(30)

    #==========================================================================#
    # Change this value to change the gear
    #==========================================================================#
    gear.data = 1

    while not rospy.is_shutdown():
        gear_pub.publish(gear)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_gear()
    except rospy.ROSInterruptException:
        pass
