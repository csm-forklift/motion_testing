/**
 * This code allows you to read the pedal signal as an analog read value
 * 
 * Pin Layout
 * Arduino        Pedal
 * A0             Signal 1 (blue)
 * GND            GND (yellow)
 */

// ROS Includes
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

//===== Pins =====//
const int SIGNAL_PIN = A0;

//===== Parameters =====//
const int DELAY_TIME = 32; // msec

//===== ROS Objects =====//
ros::NodeHandle nh;
std_msgs::Int16 raw_data;
ros::Publisher raw_pedal_pub("accelerator_node/pedal/raw_signal", &raw_data);

void setup() {
  // Set up ROS
  nh.initNode();
  nh.advertise(raw_pedal_pub);
  
  // Set up pins
  pinMode(SIGNAL_PIN, INPUT);
  
  // Begin Serial connection
  Serial.begin(57600);
}

void loop() {
  // Read data from the analog pins
  int data = analogRead(SIGNAL_PIN);

  // Convert to ROS message type
  raw_data.data = data;

  // Publish message
  raw_pedal_pub.publish(&raw_data);

  nh.spinOnce();
  delay(DELAY_TIME);
}
