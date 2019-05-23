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
        self.angle = 0 # rad, starting angle before joystick command received
        # Max and Min angles to turn of the velocity if they are reached
        self.max_angle = 2*math.pi
        self.min_angle = -2*math.pi

        self.max_accel_scale = 0.1
        self.max_vel_scale = -0.75 # negative value is used to reverse the steering direction, makes right direction on analog stick equal right turn going forward

        self.moving = False # indicates whether the motor is currently moving
        self.repeats = 0 # adds the number of times the motor is seen as not moving
        self.max_repeats = 5 # the maximum number of times the motor can be seen as not moving before reseting
        self.ramp_up = False # indicates whether the ramp up procedure should be done (follows a stall event)
        self.ramp_time = 3 # number of seconds to ramp up to full velocity again
        self.ramp_start = time.time()

        #=========================#
        # Create ROS Node Objects
        #=========================#
        rospy.init_node("steering_controller")
        rospy.on_shutdown(self.close) # shuts down the Phidget properly
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size = 1)
        self.position_pub = rospy.Publisher("~motor_position", Float64, queue_size = 10)
        self.velocity_pub = rospy.Publisher("~motor_velocity", Float64, queue_size = 10)
        self.moving_sub = rospy.Subscriber("/steering_node/motor/is_moving", Bool, self.moving_callback, queue_size = 3)

        # Specify general parameters
        self.rl_axes = 3
        self.deadman_button = rospy.get_param("~deadman", 4)
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

        # Set acceleration (currently set to 75% of max)
        self.ch.setAcceleration(self.max_accel_scale*self.ch.getMaxAcceleration())

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

    #===================================#
    # Velocity Set Function
    # * change this function whenever you use a different source for setting the
    # * velocity besides the controller.
    #===================================#
    def joy_callback(self, msg):
        # Check if deadman switch is pressed
        if (msg.buttons[self.deadman_button]):
            # check if the current angle is beyond the bounds
            self.angle = self.ch.getPosition()
            # Read right joystick analog "Left/Right" value
            # (only go to 90% ov maximum velocity so it doesn't stall out)
            self.velocity = self.max_vel_scale*self.scale_angle*msg.axes[self.rl_axes]*self.ch.getMaxVelocityLimit()

            # DEBUG: Print
            # print("Angle: " + str(self.angle))
            # print("Velocity: " + str(self.angle))
            # print("Max: " + str(self.max_angle) + ", Min: " + str(self.min_angle))

            #===== Uses Min/Max =====#
            # if not ((self.angle > self.max_angle and self.velocity > 0) or (self.angle < self.min_angle and self.velocity < 0)):
            #     try:
            #         self.ch.setVelocityLimit(self.velocity)
            #     except PhidgetException as e:
            #         DisplayError(e)
            # else:
            #     try:
            #         self.ch.setVelocityLimit(0)
            #     except PhidgetException as e:
            #         DisplayError(e)

            #===== No Min/Max considered =====#
            try:
                # Check if the motor has stopped moving even though it has a velocity command (this means it has stalled and needs to be reset)
                # self.ch.setVelocityLimit(self.velocity)

                print("Moving: %d" % self.moving)

                if not self.ramp_up:
                    if (self.ch.getVelocity() != 0 and self.moving == False):
                        self.repeats += 1
                        if (self.repeats > self.max_repeats):
                            self.ramp_up = True
                            self.ramp_start = time.time()
                        else:
                            self.ch.setVelocityLimit(self.velocity)
                    else:
                        self.repeats = 0
                        self.ch.setVelocityLimit(self.velocity)
                else:
                    scale = min((time.time() - self.ramp_start)/self.ramp_time, 1)
                    self.ch.setVelocityLimit(scale*self.velocity)
                    if (scale >= 1):
                        self.ramp_up = False
            except PhidgetException as e:
                DisplayError(e)

    def moving_callback(self, msg):
        # Update 'moving' to indicate whether the motor is moving or not
        self.moving = msg.data

def main():
    steering_controller = SteeringController()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
