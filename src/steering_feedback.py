#!/usr/bin/env python

import sys
import traceback
import time
import math
import rospy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Joy
from Phidget22.Devices.Stepper import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *

#=================================================#
# Phidget Helper Functions
#=================================================#
class EndProgramSignal(Exception):
    def __init__(self, value):
        self.value = str(value)

def DisplayError(e):
    sys.stderr.write("Desc: " + e.details + "\n")

    if (e.code == ErrorCode.EPHIDGET_WRONGDEVICE):
        sys.stderr.write("\tThis error commonly occurs when the Phidget function you are calling does not match the class of the channel that called it.\n"
                        "\tFor example, you would get this error if you called a PhidgetVoltageInput_* function with a PhidgetDigitalOutput channel.")
    elif (e.code == ErrorCode.EPHIDGET_NOTATTACHED):
        sys.stderr.write("\tThis error occurs when you call Phidget functions before a Phidget channel has been opened and attached.\n"
                        "\tTo prevent this error, ensure you are calling the function after the Phidget has been opened and the program has verified it is attached.")
    elif (e.code == ErrorCode.EPHIDGET_NOTCONFIGURED):
        sys.stderr.write("\tThis error code commonly occurs when you call an Enable-type function before all Must-Set Parameters have been set for the channel.\n"
                        "\tCheck the API page for your device to see which parameters are labled \"Must be Set\" on the right-hand side of the list.")

def PrintOpenErrorMessage(e, ph):
    sys.stderr.write("Runtime Error -> Opening Phidget Channel: \n\t")
    DisplayError(e)
    if(e.code == ErrorCode.EPHIDGET_TIMEOUT):
        sys.stderr.write("\nThis error commonly occurs if your device is not connected as specified, "
                         "or if another program is using the device, such as the Phidget Control Panel.\n")
        sys.stderr.write("\nIf your Phidget has a plug or terminal block for external power, ensure it is plugged in and powered.\n")
        if(     ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGEINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGERATIOINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALOUTPUT
        ):
            sys.stderr.write("\nIf you are trying to connect to an analog sensor, you will need to use the "
                              "corresponding VoltageInput or VoltageRatioInput API with the appropriate SensorType.\n")

        if(ph.getIsRemote()):
            sys.stderr.write("\nEnsure the Phidget Network Server is enabled on the machine the Phidget is plugged into.")


class SteeringController():
    def __init__(self):
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
        self.max_angle = 75*(math.pi/180.)
        self.min_angle = -75*(math.pi/180.)
        self.velocity_tolerance = 0.01

        # DEBUG: print max angle values used
        print("[steering_feedback] Bounding setpoint angles to, Max: {0:0.3f} ({1:0.3f} deg) Min: {2:0.3f} ({3:0.3f} deg)".format(self.max_angle, self.max_angle*(180/math.pi), self.min_angle, self.min_angle*(180/math.pi)))

        self.max_accel_scale = 0.001
        self.max_vel_scale = -0.65 # negative value is used to reverse the steering direction, makes right direction on analog stick equal right turn going forward

        #===============================================================#
        # These parameters are used in the stall detection and handling
        #===============================================================#
        # Tuning parameters
        self.max_repeats = 5 # the maximum number of times the motor can be seen as not moving before reseting
        self.ramp_time_vel = 1 # number of seconds to ramp up to full velocity again
        self.ramp_time_accel = 1 # number of seconds to ramp up to full acceleration again

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
        rospy.init_node("steering_feedback")
        rospy.on_shutdown(self.close) # shuts down the Phidget properly
        self.setpoint_sub = rospy.Subscriber("/steering_node/angle_setpoint", Float64, self.setpoint_callback, queue_size = 1)
        self.position_pub = rospy.Publisher("~motor/position", Float64, queue_size = 10)
        self.velocity_pub = rospy.Publisher("~motor/velocity", Float64, queue_size = 10)
        self.moving_sub = rospy.Subscriber("/steering_feedback/motor/is_moving", Bool, self.moving_callback, queue_size = 3)
        self.angle_sub = rospy.Subscriber("/steering_node/filtered_angle", Float64, self.angle_callback, queue_size = 3)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size = 1)
        # Run 'spin' loop at 30Hz
        self.rate = rospy.Rate(30)

        # Specify general parameters
        self.rl_axes = 3
        self.deadman_button = rospy.get_param("~deadman", 4)
        # Indicates whether the deadman switch has been pressed on the joystick
        # It must be pressed in order for the system to be able to start running
        self.deadman_on = False
        self.timeout = rospy.get_param("~timeout", 1) # number of seconds allowed since the last setpoint message before sending a 0 command
        self.timeout_start = time.time()
        self.scale_angle = rospy.get_param("~scale_angle", 1)
        self.scale_angle = min(self.scale_angle, 1)
        print("Deadman button: " + str(self.deadman_button))
        print("Scale angle: " + str(self.scale_angle))

        #================================#
        # Create phidget stepper channel
        #================================#
        try:
            self.ch = Stepper()
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Creating Stepper")
            raise
        except RuntimeError as e:
            sys.stderr.write("Runtime Error -> Creating Stepper")
            raise

        self.ch.setDeviceSerialNumber(522972)
        self.ch.setChannel(0)

        # Set handlers, these are called when certain Phidget events happen
        print("\n--------------------------------------")
        print("Starting up Phidget controller")
        print("* Setting OnAttachHandler...")
        self.ch.setOnAttachHandler(self.onAttachHandler)

        print("* Setting OnDetachHandler...")
        self.ch.setOnDetachHandler(self.onDetachHandler)

        print("* Setting OnErrorHandler...")
        self.ch.setOnErrorHandler(self.onErrorHandler)

        print("* Setting OnPositionChangeHandler...")
        self.ch.setOnPositionChangeHandler(self.onPositionChangeHandler)

        print("* Setting OnVelocityChangeHandler...")
        self.ch.setOnVelocityChangeHandler(self.onVelocityChangeHandler)

        # Attach to Phidget
        print("* Opening and Waiting for Attachment...")
        try:
            self.ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, self.ch)
            raise EndProgramSignal("Program Terminated: Open Failed")

        # Set Rescale Factor
        # (pi rad / 180 deg) * (1.8deg/step * (1/16) step) / (Gear Ratio = 26 + (103/121))
        self.rescale_factor = (math.pi/180.)*(1.8/16)/(26.+(103./121.))
        self.ch.setRescaleFactor(self.rescale_factor) # converts steps to radians

        # Set control mode (You must either uncomment the line below or do not set the control mode at all and leave let it be the default of 0, or "step" mode. Setting self.ch.setControlMode(ControlMode.CONTROL_MODE_STEP) does not work for some reason)
        if (self.control_mode == 1):
            self.ch.setControlMode(ControlMode.CONTROL_MODE_RUN)

        # Define max acceleration and velocity
        self.max_velocity = self.max_vel_scale*self.ch.getMaxVelocityLimit()
        self.max_acceleration = self.max_accel_scale*self.ch.getMaxAcceleration()
        self.ch.setAcceleration(self.max_acceleration)

        #===============#
        # Run main loop
        #===============#
        # self.mainLoop()
        self.spin()

    def onAttachHandler(self, channel):
        ph = channel

        try:
            # Get channel info
            channelClassName = ph.getChannelClassName()
            serialNumber = ph.getDeviceSerialNumber()
            channel_num = ph.getChannel()

            # DEBUG: print channel info
            print("\nAttaching Channel")
            print("* Channel Class: " + channelClassName + "\n* Serial Number: " + str(serialNumber) + "\n* Channel: " + str(channel_num) + "\n")

            # Set data time interval
            interval_time = 32 # ms (this will publish at roughly 30Hz)
            print("Setting DataInterval to %ims" % interval_time)
            try:
                ph.setDataInterval(interval_time)
            except PhidgetException as e:
                sys.stderr.write("Runtime Error -> Setting DataInterval\n")
                return

            # Engage stepper
            print("Engaging Stepper")
            try:
                ph.setEngaged(True)
            except PhidgetException as e:
                sys.stderr.write("Runtime Error -> Setting Engaged\n")
                return

        except PhidgetException as e:
            print("Error in attach event:")
            traceback.print_exc()
            return

    def onDetachHandler(self, channel):
        ph = channel

        try:
            # Get channel info
            channelClassName = ph.getChannelClassName()
            serialNumber = ph.getDeviceSerialNumber()
            channel_num = ph.getChannel()

            # DEBUG: print channel info
            print("\nDetaching Channel")
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) + "\n\t-> Channel: " + str(channel_num) + "\n")

        except PhidgetException as e:
            print("\nError in Detach Event:")
            traceback.print_exc()
            return

    def onErrorHandler(self, channel, errorCode, errorString):
        sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")

    def onPositionChangeHandler(self, channel, position):
        self.position_pub.publish(Float64(position))

    def onVelocityChangeHandler(self, channel, velocity):
        self.velocity_pub.publish(Float64(velocity))

    def close(self):
        print("\n" + 30*"-")
        print("Closing down Phidget controller")
        self.ch.setOnPositionChangeHandler(None)
        print("Cleaning up...")
        self.ch.close()
        print("Exiting...")
        return 0

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
        if (self.deadman_on and (time.time() - self.timeout_start) < self.timeout):
            # Determine direction
            error = self.angle_setpoint - self.angle

            if not (error == 0):
                direction = abs(error)/error
            else:
                direction = 1

            if (abs(error) > self.angle_tolerance):
                self.velocity = -direction*self.scale_angle*self.max_velocity
            else:
                self.velocity = 0

            #===== Stall Check and Set Velocity =====#
            try:
                # Check if the system is stalled
                if (self.check_stall()):
                    # Initiate "ramp-up" mode
                    self.reset_rampup()

                if (self.operation_mode == 0):
                    # Normal mode
                    self.ch.setVelocityLimit(self.velocity)
                else:
                    # Ramp-up mode
                    t_curr = time.time()
                    scale_vel = min((t_curr - self.ramp_start)/self.ramp_time_vel, 1)**3
                    scale_accel = min((t_curr - self.ramp_start)/self.ramp_time_accel, 1)
                    scale_accel = max(scale_accel, 0.001)

                    # DEBUG: print accel scaling
                    #print("t_curr: %f" % t_curr)
                    #print("time diff: %f" % (t_curr - self.ramp_start))
                    # print("accel scale: %f" % scale_accel)
                    # print("vel scale: %f" % scale_vel)
                    # print("Accel: %f" % (scale_accel*self.max_acceleration))
                    # print("Vel: %f" % (scale_vel*self.velocity))

                    self.ch.setAcceleration(scale_accel*self.max_acceleration)
                    self.ch.setVelocityLimit(scale_vel*self.velocity)

                    # When ramping has finished resume normal operation
                    if (scale_vel == 1 and scale_accel == 1):
                        self.operation_mode = 0
            except PhidgetException as e:
                DisplayError(e)
        else:
            self.ch.setVelocityLimit(0)

    def setpoint_callback(self, msg):
        # Read in new setpoint and saturate against the bounds
        self.angle_setpoint = min(msg.data, self.max_angle)
        self.angle_setpoint = max(self.angle_setpoint, self.min_angle)
        self.timeout_start = time.time()

    def angle_callback(self, msg):
        self.angle = msg.data

    def moving_callback(self, msg):
        # Update 'moving' to indicate whether the motor is moving or not
        self.moving = msg.data

    def check_stall(self):
        stalled = False
        if (abs(self.ch.getVelocity()) > self.velocity_tolerance and self.moving == False):
            self.repeats += 1
            if (self.repeats > self.max_repeats):
                stalled = True
        else:
            self.repeats = 0

        return stalled

    def reset_rampup(self):
        if (self.operation_mode == 0):
            self.ch.setVelocityLimit(0)
        self.ramp_start = time.time()
        self.operation_mode = 1

    def joystick_callback(self, msg):
        if (msg.buttons[self.deadman_button]):
            # The system can begin driving
            self.deadman_on = True
        else:
            self.deadman_on = False

def main():
    steering_controller = SteeringController()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
