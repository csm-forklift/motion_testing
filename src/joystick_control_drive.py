#!/usr/bin/env python

'''
Uses the Logitech F710 joystick controller to send the control signals to the
velocity and steering nodes for the forklift. The left joystick is used to
control the velocity with its vertical motion and the right joystick controls
the steering angle with its horizontal motion.

The steering angle has two modes of operation. One is absolute, where the min
and max angle are represented by maximum left and right. So you must hold the
joystick at the desired angle to get the precise steering angle. The other mode
treats the joystick as relative motion, so it represents a change in the
steering angle. The harder the joystick is angled, the faster the steering angle
with change. If the joystick is brought back to its 0 position, then the
steering angle will remain constant at is current position.
'''

import rospy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Joy
import math
import time


class JoystickController:
    def __init__(self):
        #===== Initialize ROS Objects =====#
        rospy.init_node("joystick_controller")

        #===== Set Parameters =====#
        self.angle_max = rospy.get_param("~angle_max", 75*(math.pi/180.))
        self.vel_max = rospy.get_param("~vel_max", 0.5)
        self.steering_mode = rospy.get_param("~steering_mode", "relative")
        self.manual_deadman_button = rospy.get_param("~manual_deadman", 4)
        self.manual_deadman_on = False
        self.autonomous_deadman_button = rospy.get_param("~autonomous_deadman_button", 5)
        self.autonomous_deadman_on = False
        self.timeout = rospy.get_param("~timeout", 1)
        self.timeout_start = time.time()

        #===== Publishers and Subscribers =====#
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size=1)
        self.vel_setpoint_pub = rospy.Publisher("/velocity_node/velocity_setpoint", Float64, queue_size=1)
        self.angle_setpoint_pub = rospy.Publisher("/steering_node/angle_setpoint", Float64, queue_size=1)
        self.pedal_switch_pub = rospy.Publisher("/velocity_node/pedal_switch", Bool, queue_size=1)

        if self.steering_mode not in ["relative", "absolute"]:
            rospy.loginfo("steering_mode value: '%s', is not a valid option. Must be either 'relative' or 'absolute'. Setting to 'relative'." % self.steering_mode)
            self.steering_mode = "relative"
        # If the mode is set to "relative", this parameter is scaled by the
        # joystick value and then used to update the current angle based on the
        # update rate time.
        self.angle_velocity = 1

        # Set the axes used for each setpoint
        self.vel_axes = 1 # + = joystick up
        self.angle_axes = 3 # + = joystick left

        #===== Initialize Variables =====#
        self.vel_scale = 0
        self.velocity = 0
        self.velocity_msg = Float64()
        self.angle_scale = 0
        self.angle = 0
        self.angle_msg = Float64()

        self.delta_t = (1/30.)
        self.rate = rospy.Rate(1/self.delta_t)

    def spin(self):
        while not rospy.is_shutdown():
            if (self.manual_deadman_on and (time.time() - self.timeout_start) < self.timeout):
                # Calculate the velocity based on joystick input
                self.velocity = self.vel_scale*self.vel_max

                # Calculate the steering angle based on joystick input
                # (remember, + = right, - = left, when facing forwards)
                if (self.steering_mode == "relative"):
                    self.angle += -self.angle_scale*self.angle_velocity*self.delta_t
                    # Bound the angle
                    self.angle = min(self.angle, self.angle_max)
                    self.angle = max(self.angle, -self.angle_max)
                else:
                    # Must be negated because the joystick considers left as positive
                    # but the steering angle as determined by the "backwards" driving
                    # convention used in our system makes left as negative
                    self.angle = -self.angle_scale*self.angle_max

                # Publish the messages
                self.velocity_msg.data = self.velocity
                self.angle_msg.data = self.angle
                self.vel_setpoint_pub.publish(self.velocity_msg)
                self.angle_setpoint_pub.publish(self.angle_msg)
            elif (self.autonomous_deadman_on and (time.time() - self.timeout_start) < self.timeout):
                # Send no command
                pass
            else:
                # Only set the velocity to 0, leave the angle where it is already at
                self.velocity = 0

                # Publish the messages
                self.velocity_msg.data = self.velocity
                self.angle_msg.data = self.angle
                self.vel_setpoint_pub.publish(self.velocity_msg)
                self.angle_setpoint_pub.publish(self.angle_msg)

            self.rate.sleep()

    def joystick_callback(self, msg):
        # Update timeout time
        self.timeout_start = time.time()

        # Store the velocity scaling value
        self.vel_scale = msg.axes[self.vel_axes]
        # Bound it to be positive
        self.vel_scale = max(self.vel_scale, 0)

        # Store the angle scaling value
        self.angle_scale = msg.axes[self.angle_axes]

        if (msg.buttons[self.manual_deadman_button]):
            self.manual_deadman_on = True
            pedal_on = Bool()
            pedal_on.data = True
            self.pedal_switch_pub.publish(pedal_on)
        else:
            self.manual_deadman_on = False
            pedal_on = Bool()
            pedal_on.data = False
            self.pedal_switch_pub.publish(pedal_on)

        if (msg.buttons[self.autonomous_deadman_button]):
            self.autonomous_deadman_on = True
        else:
            self.autonomous_deadman_on = False

        # Turn the accelerator relay ON if either of the deadman switches are pressed.
        pedal_on = Bool()
        if (self.manual_deadman_on or self.autonomous_deadman_on):
            pedal_on.data = True
        else:
            pedal_on.data = False
        self.pedal_switch_pub.publish(pedal_on)

if __name__ == "__main__":
    try:
        joystick_controller = JoystickController()
        joystick_controller.spin()
    except rospy.ROSInterruptException:
        pass