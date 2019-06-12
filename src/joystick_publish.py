#!/usr/bin/env python

'''
Publishes a joystick controller message to simulate pressing the deadman button, without requiring the controller. SHOULD ONLY BE USED WITH SIMULATION, NOT ON THE REAL SYSTEM.
'''


import rospy
from sensor_msgs.msg import Joy

class JoystickPublish:
    def __init__(self):
        #===== Set up ROS parameters and objects =====#
        rospy.init_node("joystick_publish")
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=3)
        self.rate = rospy.Rate(30)

        # Message
        self.joy_msg = Joy()
        self.joy_msg.buttons = [0,0,0,0,1] # button index 4 is the deadman

    def spin(self):
        '''
        Continue publishing a joystick message to simulate pressing the deadman button.
        '''
        while not rospy.is_shutdown():
            #===== Publish Joy message =====#
            self.joy_pub.publish(self.joy_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        joystick_publish = JoystickPublish()
        joystick_publish.spin()
    except rospy.ROSInterruptException:
        pass
