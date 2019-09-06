#!/usr/bin/env python

'''
Steering node designed to work with an A4988 stepper driver board.
'''

import sys
import traceback
import time
import math
import rospy
from std_msgs.msg import Float64, Float32, Bool, Int8, Int32
from sensor_msgs.msg import Joy


class SteeringController():
    def __init__(self):
        rospy.init_node("steering_node")
        # Set control mode
        # 0 = step mode (target position)
        # 1 = run mode (target velocity)
        self.control_mode = 1
        self.velocity = 0 # rad/s, starting velocity
        self.angle = 0 # current steering angle
        self.angle_setpoint = 0 # current steering angle setpoint

        # TODO: makes these ROS parameters
        self.angle_tolerance = 0.02 # stops moving the motor when the angle is within +/- the tolerance of the desired setpoint
        # Max and Min angles to turn of the velocity if they are reached
        self.max_angle = rospy.get_param("/forklift/steering/max_angle", 75*(math.pi/180.))
        self.min_angle = rospy.get_param("/forklift/steering/min_angle", -75*(math.pi/180.))
        self.max_vel_scale = rospy.get_param("~max_vel_scale", 0.17)
        print("[%s]: using max_vel_scale: %0.04f" % (rospy.get_name(), self.max_vel_scale))
        self.ramp_time_vel = rospy.get_param("~ramp_time_vel", 1.5) # number of seconds to ramp up to full velocity again

        # DEBUG: print max angle values used
        print("[steering_node] Bounding setpoint angles to, Max: {0:0.3f} ({1:0.3f} deg) Min: {2:0.3f} ({3:0.3f} deg)".format(self.max_angle, self.max_angle*(180/math.pi), self.min_angle, self.min_angle*(180/math.pi)))

        self.velocity_current = 0 # current velocity from the motor controller
        self.gear = 0 # current gear of the forklift, should only steer if not in neutral (gear = 0)

        #===============================================================#
        # These parameters are used in the stall detection and handling
        #===============================================================#
        # Tuning parameters
        self.max_repeats = 5 # the maximum number of times the motor can be seen as not moving before reseting
        self.ramp_time_accel = 1.5 # number of seconds to ramp up to full acceleration again

        # Operation states
        self.moving = False # indicates whether the motor is currently moving
        self.repeats = 0 # adds the number of times the motor is seen as not moving
        # indicates whether the velocity command is sent as normal or if the
        # ramp-up prodecure should be used.
        # 0 = normal mode
        # 1 = ramp-up mode
        self.operation_mode = 0
        self.ramp_start = time.time() # time when the ramp up procedure began

        #=========================#
        # Create ROS Node Objects
        #=========================#
        # Specify general parameters
        self.rl_axes = 3
        self.manual_deadman_button = rospy.get_param("~manual_deadman", 4)
        # Indicates whether the deadman switch has been pressed on the joystick
        # It must be pressed in order for the system to be able to start running
        self.manual_deadman_on = False
        self.autonomous_deadman_button = rospy.get_param("~autonomous_deadman", 5)
        self.autonomous_deadman_on = False
        self.timeout = rospy.get_param("~timeout", 1) # number of seconds allowed since the last setpoint message before sending a 0 command
        self.timeout_start = time.time()
        self.scale_angle = rospy.get_param("~scale_angle", 1)
        self.scale_angle = min(self.scale_angle, 1)
        print("Manual deadman button: " + str(self.manual_deadman_button))
        print("Autonomous deadman button: " + str(self.autonomous_deadman_button))
        print("Scale angle: " + str(self.scale_angle))

        # Publishers and Subscribers
        self.velocity_msg = Float32()
        self.velocity_pub = rospy.Publisher("~motor/velocity", Float32, queue_size = 3)
        self.enable_msg = Bool()
        self.enable_pub = rospy.Publisher("~motor/enable", Bool, queue_size = 3)
        self.setpoint_sub = rospy.Subscriber("/steering_node/angle_setpoint", Float64, self.setpoint_callback, queue_size = 1)
        self.moving_sub = rospy.Subscriber("/steering_node/motor/is_moving", Bool, self.moving_callback, queue_size = 3)
        self.angle_sub = rospy.Subscriber("/steering_node/filtered_angle", Float64, self.angle_callback, queue_size = 3)
        self.gear_sub = rospy.Subscriber("/velocity_node/gear", Int8, self.gear_callback, queue_size = 3)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size = 1)
        # Run 'spin' loop at 30Hz
        self.rate = rospy.Rate(30)

        # Set Rescale Factor (radians/step)
        # (pi rad / 180 deg) * (1.8deg/step) / (Gear Ratio = 26 + (103/121))
        self.rescale_factor = (math.pi/180.)*(1.8)/(26.+(103./121.))
        self.velocity_tolerance = 0.01/self.rescale_factor

        # Define max acceleration and velocity
        self.phidget_max_velocity = 250000.0/16 # steps/sec
        self.max_velocity = self.max_vel_scale*self.phidget_max_velocity

        #===============#
        # Run main loop
        #===============#
        # self.mainLoop()
        self.spin()

    def spin(self):
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    #===================================#
    # Velocity Set Function
    # * change this function whenever you use a different source for setting the
    # * velocity besides the controller.
    #===================================#
    def control_loop(self):
        # Check if deadman switch is pressed
        if ((self.manual_deadman_on or self.autonomous_deadman_on) and (time.time() - self.timeout_start) < self.timeout and (self.gear != 0)):
            # Turn on the motors
            self.enable_msg.data = True
            self.enable_pub.publish(self.enable_msg)

            # Determine direction
            error = self.angle_setpoint - self.angle

            if not (error == 0):
                direction = abs(error)/error
            else:
                direction = 1

            if (abs(error) > self.angle_tolerance):
                self.velocity = direction*self.scale_angle*self.max_velocity
            else:
                self.velocity = 0

            #===== Stall Check and Set Velocity =====#
            # Check if the system is stalled
            if (self.check_stall()):
                # Initiate "ramp-up" mode
                self.reset_rampup()

            if (self.operation_mode == 0):
                # Normal mode
                self.velocity_msg.data = self.velocity
                self.velocity_pub.publish(self.velocity_msg)

                # Scale down the acceleration as the velocity increases
                # # (Linear)
                # accel_vel_scale = (self.max_velocity - abs(self.velocity_current))/(self.max_velocity)
                # accel_vel_scale = min(accel_vel_scale, 1)
                # accel_vel_scale = max(accel_vel_scale, 0)

                # (Inverse)
                # parameters
                unchanged_length = 0.75 # increase this value to increase the range where the accelerations remains unreduced
                final_scale = 10 # increase this value to decrease the final scale value at max velocity

                # Set acceleration
                print("Current velocity: %f, max: %f" % (self.velocity, self.max_velocity))

            else:
                # Ramp-up mode
                t_curr = time.time()
                scale_vel = min((t_curr - self.ramp_start)/self.ramp_time_vel, 1)**3

                # DEBUG: print accel scaling
                print("t_curr: %f" % t_curr)
                print("time diff: %f" % (t_curr - self.ramp_start))
                print("vel scale: %f" % scale_vel)
                print("Vel: %f" % (scale_vel*self.velocity))

                self.velocity_msg.data = scale_vel*self.velocity
                self.velocity_pub.publish(self.velocity_msg)

                # When ramping has finished resume normal operation
                if (scale_vel == 1):
                    self.operation_mode = 0

        else:
            self.velocity_msg.data = 0
            self.velocity_pub.publish(self.velocity_msg)

            # Turn off the motor
            self.enable_msg.data = False
            self.enable_pub.publish(self.enable_msg)

    def setpoint_callback(self, msg):
        # Read in new setpoint and saturate against the bounds
        self.angle_setpoint = min(msg.data, self.max_angle)
        self.angle_setpoint = max(self.angle_setpoint, self.min_angle)

    def angle_callback(self, msg):
        # Read the current steering angle
        self.angle = msg.data

    def moving_callback(self, msg):
        # Update 'moving' to indicate whether the motor is moving or not
        self.moving = msg.data

    def gear_callback(self, msg):
        # Update gear
        self.gear = msg.data

    def check_stall(self):
        stalled = False
        if (abs(self.velocity) > self.velocity_tolerance and self.moving == False):
            self.repeats += 1
            if (self.repeats > self.max_repeats):
                stalled = True
        else:
            self.repeats = 0

        return stalled

    def reset_rampup(self):
        if (self.operation_mode == 0):
            self.velocity_msg.data = 0
            self.velocity_pub.publish(self.velocity_msg)
        self.ramp_start = time.time()
        self.operation_mode = 1

    def joystick_callback(self, msg):
        # Update timeout time
        self.timeout_start = time.time()

        # One of these buttons must be on for this node to send a steering command
        if (msg.buttons[self.manual_deadman_button]):
            self.manual_deadman_on = True
        else:
            self.manual_deadman_on = False

        if (msg.buttons[self.autonomous_deadman_button]):
            self.autonomous_deadman_on = True
        else:
            self.autonomous_deadman_on = False

def main():
    steering_controller = SteeringController()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
