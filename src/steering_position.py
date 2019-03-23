#!/usr/bin/env python

'''
This node handles receiving the joystick inputs (the right analog stick) and
converts the value into a steering angle position and moves the motor to that
position.
'''

import sys
import traceback
import time
import math
import rospy
from std_msgs.msg import Float64
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
        self.control_mode = 0 # needs to be position for this code
        self.angle = 0 # starting angle before joystick command received

        #=========================#
        # Create ROS Node Objects
        #=========================#
        rospy.init_node("steering_position")
        rospy.on_shutdown(self.close) # shuts down the Phidget properly
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size = 1)
        self.position_pub = rospy.Publisher("~motor_position", Float64, queue_size = 10)
        self.velocity_pub = rospy.Publisher("~motor_velocity", Float64, queue_size = 10)

        # Specify general parameters
        self.rl_axes = 3
        self.deadman_button = rospy.get_param("~deadman", 4)
        self.scale_angle = rospy.get_param("~scale_angle", 2*math.pi)
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


        #====================#
        # Set Parameters
        #====================#
        # Set Rescale Factor
        # (pi rad / 180 deg) * (1.8deg/step * (1/16) step) / (Gear Ratio = 26 + (103/121))
        self.rescale_factor = (math.pi/180.)*(1.8/16)/(26.+(103./121.))
        self.ch.setRescaleFactor(self.rescale_factor) # converts steps to radians

        # Set max velocity for position control
        if (self.control_mode == 0):
            max_speed = 250000 # (1/16) steps / sec
            max_speed *= self.rescale_factor
            self.ch.setVelocityLimit(max_speed)


        #===============#
        # Run main loop
        #===============#
        # self.mainLoop()
        rospy.spin()

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

    def joy_callback(self, msg):
        # Check if deadman switch is pressed
        if (msg.buttons[self.deadman_button]):
            # Read right joystick analog "Left/Right" value
            self.angle = self.scale_angle*msg.axes[self.rl_axes]

        max_position = self.ch.getMaxPosition()
        min_position = self.ch.getMinPosition()
        if (self.angle > max_position):
            self.angle = max_position
            print("Desired position greater than max, setting to max value of %.2f" % max_position)
        elif (self.angle < min_position):
            self.angle = min_position
            print("Desired position less than min, setting to min value of %.2f" % min_position)

        try:
            self.ch.setTargetPosition(self.angle)
        except PhidgetException as e:
            DisplayError(e)


def main():
    steering_controller = SteeringController()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
