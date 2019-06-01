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
        rospy.init_node("velocity_pid")
        self.velocity_sub = rospy.Subscriber("velocity_node/velocity", Float64, self.velocity_callback, queue_size=3)
        self.setpoint_sub = rospy.Subscriber("velocity_node/velocity_setpoint", Float64, self.setpoint_callback, queue_size=3)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback)
        self.control_pub = rospy.Publisher("velocity_node/pedal_pwm", UInt8, queue_size=5)

        self.Kp = rospy.get_param("~Kp", 1) # proportional gain
        self.Ki = rospy.get_param("~Ki", 0) # integral gain
        self.Kd = rospy.get_param("~Kd", 0) # derivative gain
        self.integral_mode = rospy.get_param("~integral_mode", "window")
        if self.integral_mode not in ["window", "continuous"]:
            rospy.loginfo("integral_mode value: '%s', is not a valid option. Must be either 'window' or 'continuous'. Setting to 'window'." % self.integral_mode)
            self.integral_mode = "window"

        self.deadman_button = rospy.get_param("~deadman", 4)
        self.deadman_on = False # this must be switched to 'True' to publish a non-zero command signal
        self.timeout = rospy.get_param("~timeout", 1) # number of seconds allowed since the last setpoint message before sending a 0 command
        self.timeout_start = time.time()
        self.window_size = rospy.get_param("~window_size", 30)
        self.delta_t = rospy.get_param("~delta_t", 1/30.) # defaults to run at 30Hz
        self.output_max = rospy.get_param("~output_max", 255)
        self.output_min = rospy.get_param("~output_min", 0)

        # DEBUG: print out bounding values
        print("[velocity_pid] Bounding control output to, Max: {0:d}, Min: {1:d}".format(self.output_max, self.output_min))

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
            if (self.deadman_on and (time.time() - self.timeout_start) < self.timeout):
                #===== Calculate the control variable using the current process measurement and setpoint =====#
                # Calculate error
                self.e_curr = self.y_setpoint - self.y_curr

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

                # Control Law
                self.u = self.Kp*self.e_curr + self.Ki*self.e_sum + self.Kd*(self.e_curr - self.e_prev)/dt

                # Update error and time terms
                self.e_prev = self.e_curr
                self.t_prev = self.t_curr

                #===== Constraints =====#
                #*** Apply constraints to the output here such as u >= 0 or apply a conversion like u = scale_factor * u. ***#
                self.u = min(self.u, self.output_max)
                self.u = max(self.u, self.output_min)
                self.u = np.uint8(self.u)
            else:
                self.u_msg.data = np.uint8(0)

            #===== Publish control variable =====#
            self.u_msg.data = self.u
            self.control_pub.publish(self.u_msg)
            self.rate.sleep()

    def velocity_callback(self, msg):
        self.y_curr = msg.data

    def setpoint_callback(self, msg):
        self.y_setpoint = msg.data
        self.timeout_start = time.time()

    def joystick_callback(self, msg):
        if (msg.buttons[self.deadman_button]):
            # The system can begin driving
            self.deadman_on = True
        else:
            self.deadman_on = False


if __name__ == "__main__":
    try:
        pid_controller = PIDController()
        pid_controller.spin()
    except rospy.ROSInterruptException:
        pass
