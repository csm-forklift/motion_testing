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
from std_msgs.msg import Float32, Float64, Bool, Int8
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
        self.clamp_scale = rospy.get_param("~clamp_scale", 0.5)
        self.steering_mode = rospy.get_param("~steering_mode", "relative")
        self.manual_deadman_button = rospy.get_param("~manual_deadman", 4)
        self.manual_deadman_on = False
        self.autonomous_deadman_button = rospy.get_param("~autonomous_deadman", 5)
        self.autonomous_deadman_on = False
        self.timeout = rospy.get_param("~timeout", 1)
        self.timeout_start = time.time()
        self.forward_button = rospy.get_param("~forward_button", 3) # "Y" button
        self.neutral_button = rospy.get_param("~neutral_button", 2) # "X" button
        self.reverse_button = rospy.get_param("~reverse_button", 0) # "A" button

        #===== Publishers and Subscribers =====#
        self.vel_setpoint_pub = rospy.Publisher("/velocity_node/velocity_setpoint", Float64, queue_size=3)
        self.angle_setpoint_pub = rospy.Publisher("/steering_node/angle_setpoint", Float64, queue_size=3)
        self.pedal_switch_pub = rospy.Publisher("/velocity_node/pedal_switch", Bool, queue_size=3)
        self.clamp_movement_pub = rospy.Publisher("/clamp_switch_node/clamp_movement", Float32, queue_size=3)
        self.clamp_grasp_pub = rospy.Publisher("/clamp_switch_node/clamp_grasp", Float32, queue_size=3)
        self.gear_pub = rospy.Publisher("/velocity_node/gear", Int8, queue_size=3)

        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size=1)
        self.gear_sub = rospy.Subscriber("/velocity_node/gear", Int8, self.gear_callback, queue_size=1) # read the most recent gear sent my the controllers

        if self.steering_mode not in ["relative", "absolute"]:
            rospy.loginfo("steering_mode value: '%s', is not a valid option. Must be either 'relative' or 'absolute'. Setting to 'relative'." % self.steering_mode)
            self.steering_mode = "relative"
        # If the mode is set to "relative", this parameter is scaled by the
        # joystick value and then used to update the current angle based on the
        # update rate time.
        self.angle_velocity = 1

        # Set the axes used for each setpoint
        self.vel_axes = 1 # + = joystick up
        self.angle_axes = 0 # + = joystick left
        self.open_axes = 3 # + = joystick up
        self.raise_axes = 4 # + = joystick left

        #===== Initialize Variables =====#
        self.vel_scale = 0
        self.velocity = 0
        self.velocity_msg = Float64()
        self.angle_scale = 0
        self.angle = 0
        self.angle_msg = Float64()
        self.open_command = 0
        self.open_msg = Float32()
        self.raise_command = 0 # remember that a negative value = upwards
        self.raise_msg = Float32()
        self.gear = 0
        self.gear_msg = Int8()

        self.delta_t = (1/30.)
        self.rate = rospy.Rate(1/self.delta_t)

    def spin(self):
        while not rospy.is_shutdown():
            if (self.manual_deadman_on and (time.time() - self.timeout_start) < self.timeout):
                # Calculate the velocity based on joystick input
                self.velocity = self.vel_scale*self.vel_max

                # If gear is "forward" make sure velocity is positive, if "reverse" it should be negative
                if (self.gear == 1):
                    self.velocity = max(0, self.velocity)
                elif (self.gear == -1):
                    self.velocity = min(0, self.velocity)
                else:
                    self.velocity = 0

                # Calculate the steering angle based on joystick input
                # (remember, + = right, - = left, when facing forwards)
                # NOTE: if gear is in "neutral" do not change the steering angle
                if (self.gear != 0):
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
                self.open_msg.data = self.clamp_scale*self.open_command
                self.raise_msg.data = self.clamp_scale*self.raise_command
                self.clamp_grasp_pub.publish(self.open_msg)
                self.clamp_movement_pub.publish(self.raise_msg)
                self.gear_msg.data = self.gear
                self.gear_pub.publish(self.gear_msg)
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

        # Store the angle scaling value
        self.angle_scale = msg.axes[self.angle_axes]

        # Store clamp opening
        self.open_command = msg.axes[self.open_axes]

        # Store clamp raise value
        self.raise_command = -msg.axes[self.raise_axes] # value is negative because a positive command sends the clamp downward, this way an upward movement on the joystick gives an upward movement on the clamp (instead of following the forklift's switch logic with forward being down and backward being up)

        # Read the gear change, favor neutral first, then forward, then reverse
        if (msg.buttons[self.neutral_button]):
            self.gear = 0
        elif (msg.buttons[self.forward_button]):
            self.gear = 1
        elif (msg.buttons[self.reverse_button]):
            self.gear = -1
        # No 'else' statement. If the buttons are not pressed, keep the last received gear command

        if (msg.buttons[self.manual_deadman_button]):
            self.manual_deadman_on = True
        else:
            self.manual_deadman_on = False

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

    def gear_callback(self, msg):
        # Update gear
        self.gear = msg.data

if __name__ == "__main__":
    try:
        joystick_controller = JoystickController()
        joystick_controller.spin()
    except rospy.ROSInterruptException:
        pass
