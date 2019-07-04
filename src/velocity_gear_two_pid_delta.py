#!/usr/bin/env python

'''
Implements a simple Discrete-Time PID controller in ROS.
'''


import rospy
from std_msgs.msg import UInt8, Float32, Float64
from sensor_msgs.msg import Joy
import numpy as np
import Queue
import time

class PIDController:
    def __init__(self):
        #===== Set up ROS parameters and objects =====#
        rospy.init_node("velocity_two_pid_delta")

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
        self.window_size = rospy.get_param("~window_size", 30)
        self.delta_t = rospy.get_param("~delta_t", 1/30.) # defaults to run at 30Hz
        self.output_max = rospy.get_param("~output_max", 255)
        self.output_min = rospy.get_param("~output_min", 0)
        self.delta_max = rospy.get_param("~delta_max", 100)
        self.delta_min = rospy.get_param("~delta_min", -20)
        self.error_tolerance = rospy.get_param("~error_tolerance", 0.1)
        self.error_exponent1 = rospy.get_param("~error_exponent1", 1.0) # exponent applied to only the error for the proportional term in the PID controller
        self.error_exponent2 = rospy.get_param("~error_exponent2", 1.0)

        # Gear direction -1 = reverse, 0 = neutral, 1 = forward
        self.gear = 0 # start in neutral until told otherwise

        # DEBUG: print out bounding values
        print("[velocity_gear_two_pid_delta] Bounding control output to, Min: {0:d}, Max: {1:d}".format(self.output_min, self.output_mas))

        # Set publishers and subscribers
        self.velocity_sub = rospy.Subscriber("velocity_node/velocity", Float64, self.velocity_callback, queue_size=3)
        self.setpoint_sub = rospy.Subscriber("velocity_node/velocity_setpoint", Float64, self.setpoint_callback, queue_size=3)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size=3)
        self.gear_sub = rospy.Subscriber("/forklift/gear", Int8, self.gear_callback, queue_size=3)
        self.control_pub = rospy.Publisher("velocity_node/pedal_pwm", UInt8, queue_size=5)

        # Rate at which to run the control loop, based on 'delta_t'
        self.rate = rospy.Rate(1/self.delta_t)

        #===== Initialize variables =====#
        # Error terms
        self.e_curr = 0
        self.e_prev = 0
        self.e_sum = 0
        self.e_sum_queue = Queue.Queue()

        # Control variable
        self.u = 0
        self.u_delta = 0
        self.u_msg = UInt8()

        # Process outputs
        self.y_setpoint = 0
        self.y_curr = 0

        # Times
        self.t_curr = time.time()
        self.t_prev = self.t_curr

    def spin(self):
        '''
        Performs the loop where the control variable is calculated based on the
        PID control law and that value is then published.
        '''
        while not rospy.is_shutdown():
            if ((self.manual_deadman_on or self.autonomous_deadman_on) and (time.time() - self.timeout_start) < self.timeout):
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
                elif (abs(self.e_curr) > self.error_tolerance):
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
                self.u_delta = min(self.u_delta, self.delta_max)
                self.u_delta = max(self.u_delta, self.delta_min)
                self.u += self.u_delta
                self.u = min(self.u, self.output_max)
                self.u = max(self.u, self.output_min)

                # DEBUG:
                print("control: %d, delta: %0.4f, error: %0.4f, vel sp: %0.4f, vel curr: %0.4f" % (self.u, self.u_delta, self.e_curr, self.y_setpoint, self.y_curr))
            else:
                self.u = 0

            #===== Publish control variable =====#
            self.u_msg.data = np.uint8(self.u)
            self.control_pub.publish(self.u_msg)
            self.rate.sleep()

    def velocity_callback(self, msg):
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


if __name__ == "__main__":
    try:
        pid_controller = PIDController()
        pid_controller.spin()
    except rospy.ROSInterruptException:
        pass
