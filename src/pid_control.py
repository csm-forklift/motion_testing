#!/usr/bin/env python

'''
Implements a simple Discrete-Time PID controller in ROS.
'''


import rospy
from std_msgs.msg import Float32, Float64
import Queue
import time

class PIDController:
    def __init__(self):
        #===== Set up ROS parameters and objects =====#
        rospy.init_node("pid_control")
        self.current_sub = rospy.Subscriber("pid_control/current_measurement", Float64, self.current_callback, queue_size=3)
        self.setpoint_sub = rospy.Subscriber("pid_control/setpoint", Float64, self.setpoint_callback, queue_size=3)
        self.control_pub = rospy.Publisher("pid_control/control_variable", Float32, queue_size=5)

        self.Kp = rospy.get_param("~Kp", 1) # proportional gain
        self.Ki = rospy.get_param("~Ki", 0) # integral gain
        self.Kd = rospy.get_param("~Kd", 0) # derivative gain
        self.integral_mode = rospy.get_param("~integral_mode", "window")
        if self.integral_mode not in ["window", "continuous"]:
            rospy.loginfo("integral_mode value: '%s', is not a valid option. Must be either 'window' or 'continuous'. Setting to 'window'." % self.integral_mode)
            self.integral_mode = "window"

        self.window_size = rospy.get_param("~window_size", 30)
        self.delta_t = rospy.get_param("~delta_t", 1/30.) # defaults to run at 30Hz

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
        self.u_msg = Float32()

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
            self.u = max(self.u, 0)

            #===== Publish control variable =====#
            self.u_msg.data = self.u
            self.control_pub.publish(self.u_msg)

            self.rate.sleep()

    def current_callback(self, msg):
        self.y_curr = msg.data

    def setpoint_callback(self, msg):
        self.y_setpoint = msg.data


if __name__ == "__main__":
    try:
        pid_controller = PIDController()
        pid_controller.spin()
    except rospy.ROSInterruptException:
        pass
