#!/usr/bin/env python

'''
Python code for reading the data from a microcontroller sending data over a
serial connection and then converts that data into a ROS message and publishes
it.
'''

import serial
import time
import rospy
from std_msgs.msg import Int16

# Begin Serial Connection
# ser = serial.Serial('/dev/cu.usbmodem14101') # for MacOS
ser = serial.Serial('/dev/ttyACM0') # for Linux
ser.baudrate = 57600

time.sleep(1)

# Begin ROS
steer_pub = rospy.Publisher("steering_node/potentiometer/raw_data", Int16, queue_size=10)
rospy.init_node("steering_serial_publish")

try:
    while not rospy.is_shutdown():
        data = int(ser.readline())
        raw_data = Int16()
        raw_data.data = data
        steer_pub.publish(raw_data)
    ser.close();
except KeyboardInterrupt:
    ser.close();
