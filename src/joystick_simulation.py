#!/usr/bin/env python
'''
Simulates sending a joystick command for when you are not able to use the
physical joystick.
'''

import rospy
from sensor_msgs.msg import Joy


class JoystickSimulation:
    def __init__(self):
        rospy.init_node("joystick_simulation")
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)
        self.rate = rospy.Rate(20)
        self.joy_msg = Joy()

    def spin(self):
        while not rospy.is_shutdown():
            # Simulates pressing the deadman
            self.joy_msg.buttons = [0,0,0,0,1,0]

            self.joy_pub.publish(self.joy_msg)
            self.rate.sleep()


try:
    joystick_simulation = JoystickSimulation()
    joystick_simulation.spin()
except rospy.ROSInterruptException:
    pass
