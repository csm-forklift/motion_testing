#!/usr/bin/env python

'''
Implements a simple Discrete-Time PID controller in ROS.
u: controller output
y: feedback measurement
'''


import rospy
from std_msgs.msg import UInt8, Int8, Float32, Float64
from sensor_msgs.msg import Joy
import numpy as np
import Queue
import time

class PIDController:
    def __init__(self):
        #===== Set up ROS parameters and objects =====#
        rospy.init_node("velocity_gear_two_pid_delta")

        # Set parameters
        self.Kp1 = rospy.get_param("~Kp1", 1) # proportional gain
        self.Ki1 = rospy.get_param("~Ki1", 0) # integral gain
        self.Kd1 = rospy.get_param("~Kd1", 0) # derivative gain
        self.Kp2 = rospy.get_param("~Kp2", 1) # proportional gain
        self.Ki2 = rospy.get_param("~Ki2", 0) # integral gain
        self.Kd2 = rospy.get_param("~Kd2", 0) # derivative gain
        self.integral_mode = rospy.get_param("~integral_mode", "window")
        if self.integral_mode not in ["window", "continuous"]:
            rospy.loginfo("integral_mode value: '%s', is not a valid option. Must be either 'window' or 'continuous'. Setting to 'window'." % self.integral_mode)
            self.integral_mode = "window"

        self.manual_deadman_button = rospy.get_param("~manual_deadman", 4)
        self.manual_deadman_on = False # this must be switched to 'True' to publish a non-zero command signal
        self.autonomous_deadman_button = rospy.get_param("~autonomous_deadman", 5)
        self.autonomous_deadman_on = False
        self.timeout = rospy.get_param("~timeout", 1) # number of seconds allowed since the last setpoint message before sending a 0 command
        self.timeout_start = time.time()
        self.velocity_timeout = rospy.get_param("~velocity_timeout", 1) # makes sure a new current velocity reading has been received within the timeout period or a 0 PWM is sent (to protect against not receiving a velocity from the canbus and thinking the current velocity is constantly at 0)
        self.velocity_timeout_start = time.time()
        self.window_size = rospy.get_param("~window_size", 30)
        self.delta_t = rospy.get_param("~delta_t", 1/30.) # defaults to run at 30Hz
        self.output_max = rospy.get_param("~output_max", 255)
        self.output_min = rospy.get_param("~output_min", 0)
        self.delta_max = rospy.get_param("~delta_max", 100)
        self.delta_min = rospy.get_param("~delta_min", -20)
        self.delta_threshold = rospy.get_param("~delta_threshold", 0.5) # minimum magnitude of u_delta
        self.error_tolerance = rospy.get_param("~error_tolerance", 0.05)
        self.error_exponent1 = rospy.get_param("~error_exponent1", 1.0) # exponent applied to only the error for the proportional term in the PID controller
        self.error_exponent2 = rospy.get_param("~error_exponent2", 1.0)

        # Gear direction -1 = reverse, 0 = neutral, 1 = forward
        self.gear = 0 # start in neutral until told otherwise

        # DEBUG: print out bounding values
        print("[velocity_gear_two_pid_delta] Bounding control output to, Min: {0:d}, Max: {1:d}".format(self.output_min, self.output_max))

        # Set publishers and subscribers
        self.control_pub = rospy.Publisher("velocity_node/pedal_pwm", UInt8, queue_size=5)

        self.velocity_sub = rospy.Subscriber("velocity_node/velocity", Float64, self.velocity_callback, queue_size=3)
        self.setpoint_sub = rospy.Subscriber("velocity_node/velocity_setpoint", Float64, self.setpoint_callback, queue_size=3)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size=3)
        self.gear_sub = rospy.Subscriber("/velocity_node/gear", Int8, self.gear_callback, queue_size=3)
        self.steering_angle_sub = rospy.Subscriber("/steering_node/filtered_angle", Float64, self.steering_angle_callback, queue_size=3)

        # Rate at which to run the control loop, based on 'delta_t'
        self.rate = rospy.Rate(1.0/self.delta_t)

        #===== Initialize variables =====#
        # Current steering angle
        self.steering_angle = 0.0

        # Error terms
        self.e_curr = 0.0
        self.e_prev = 0.0
        self.e_sum = 0.0
        self.e_sum_queue = Queue.Queue()

        # Control variable
        self.u = 0.0
        self.u_delta = 0.0
        self.u_msg = UInt8()

        # Process outputs
        self.y_setpoint = 0.0
        self.y_curr = 0.0

        # Times
        self.t_curr = time.time()
        self.t_prev = self.t_curr

    def spin(self):
        '''
        Performs the loop where the control variable is calculated based on the
        PID control law and that value is then published.
        '''
        while not rospy.is_shutdown():
            current_time = time.time()
            timeout_delta = current_time - self.timeout_start
            velocity_timeout_delta = current_time - self.velocity_timeout_start
            
            if ((self.manual_deadman_on or self.autonomous_deadman_on) and (timeout_delta < self.timeout) and (velocity_timeout_delta < self.velocity_timeout)):
                #===== Calculate the control variable using the current process measurement and setpoint =====#
                # Calculate error
                self.e_curr = self.y_setpoint - self.y_curr

                if (self.gear != 0):
                    self.e_curr = self.gear*self.e_curr

                # Calculate time difference from last command
                self.t_curr = time.time()
                dt = (self.t_curr - self.t_prev)

                # Calculate error integral
                if (self.integral_mode == "continuous"):
                    self.e_sum += self.e_curr*dt
                else:
                    self.e_sum_queue.put(self.e_curr*dt)
                    self.e_sum += self.e_curr*dt
                    if (self.e_sum_queue.qsize() > self.window_size):
                        self.e_sum -= self.e_sum_queue.get()

                #==============================================================#
                # Control Laws
                # There are two conrol laws, one for when the forklift is
                # accelerating and one for when the forklift is braking. Since
                # the forklift has different dynamics in each case, a different
                # controller is needed in each scenario.
                #==============================================================#
                if (self.y_setpoint == 0):
                    self.u_delta = self.delta_min
                elif (abs(self.e_curr) > self.error_tolerance or self.y_setpoint <= self.error_tolerance):
                    if (self.e_curr >= 0):
                        # Control Law 1
                        # Error is positive, so forklift needs to accelerate
                        self.u_delta = self.Kp1*(self.e_curr**self.error_exponent1) + self.Ki1*self.e_sum + self.Kd1*(self.e_curr - self.e_prev)/dt
                    else:
                        # Control Law 2
                        # Error is negative, so forklift needs to brake
                        e_curr_pos = abs(self.e_curr)
                        e_exponent = -(e_curr_pos**self.error_exponent2)
                        self.u_delta = self.Kp2*(e_exponent) + self.Ki2*self.e_sum + self.Kd2*(self.e_curr - self.e_prev)/dt
                else:
                    self.u_delta = 0

                # Update error and time terms
                self.e_prev = self.e_curr
                self.t_prev = self.t_curr

                #===== Constraints =====#
                #*** Apply constraints to the output here such as u >= 0 or apply a conversion like u = scale_factor * u. ***#
                # The output here is PWM going to the Arduino (range 0-255)
                self.u_delta = min(self.u_delta, self.delta_max)
                self.u_delta = max(self.u_delta, self.delta_min)

                # Scale the delta based on the steering angle (a higher steering angle should have a higher delta because the full range of the pedal has been reduced)
                self.u_delta *= 1/(np.sign(np.cos(self.steering_angle))*np.abs(np.cos(self.steering_angle)**2))

                self.u += self.u_delta

                # DEBUG:
                print("[%s]: u_delta: %f" % (rospy.get_name(), self.u_delta))

                self.u = min(self.u, self.output_max)
                self.u = max(self.u, self.output_min)

                # DEBUG:
                print("control: %d, delta: %0.4f, error: %0.4f, vel sp: %0.4f, vel curr: %0.4f" % (self.u, self.u_delta, self.e_curr, self.y_setpoint, self.y_curr))
            else:
                if (velocity_timeout_delta >= self.velocity_timeout):
                    print("[%s]: velocity message has timed out, check the CANBUS connection" % (rospy.get_name()))
                self.u = 0

            #===== Publish control variable =====#
            self.u_msg.data = np.uint8(self.u)
            self.control_pub.publish(self.u_msg)
            self.rate.sleep()

    def velocity_callback(self, msg):
        # Update velocity signal received time
        self.velocity_timeout_start = time.time()

        self.y_curr = msg.data

    def setpoint_callback(self, msg):
        self.y_setpoint = msg.data

    def joystick_callback(self, msg):
        # Update timeout time
        self.timeout_start = time.time()

        # One of these buttons must be on for this node to send a driving command
        if (msg.buttons[self.manual_deadman_button]):
            self.manual_deadman_on = True
        else:
            self.manual_deadman_on = False

        if (msg.buttons[self.autonomous_deadman_button]):
            self.autonomous_deadman_on = True
        else:
            self.autonomous_deadman_on = False

    def gear_callback(self, msg):
        # Update the gear direction
        self.gear = msg.data

    def steering_angle_callback(self, msg):
        # Get the steering angle to adjust the 'delta' value to be larger for higher steering angles
        self.steering_angle = msg.data


if __name__ == "__main__":
    try:
        pid_controller = PIDController()
        pid_controller.spin()
    except rospy.ROSInterruptException:
        pass
