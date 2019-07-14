 /**
 * This code reads in the signal from the steering potentiometer and 
 * publishes it on the raw data topic.
 * 
 * This code is intended to be used with the steering feedback nodes.
 * 
 * Pin Layout
 * Arduino    Steering Potentiometer
 * A0   ->    Signal (Black)
 * GND  ->    GND (yellow)
 * 
 * Diagram
 *         Forklift wires should remain connected to each other
 *                      Forklift
 *                X-------5V
 * Arduino
 * A0----------R??Ohm-----Signal
 *          |
 *       C0.1uF
 *          |
 * GND--------------------GND
 */

// ROS Includes
#include <ros.h>
#include <std_msgs/Int16.h>

//===== Pins =====//
const int STEERING_PIN = A0;

//===== Parameters =====//
const int DELAY_TIME = 32; // msec

//===== ROS Objects =====//
ros::NodeHandle nh;
std_msgs::Int16 raw_data;
ros::Publisher raw_pot_pub("steering_node/potentiometer/raw_data", &raw_data);

void setup() {
  // Set up ROS
  nh.initNode();
  nh.advertise(raw_pot_pub);
  
  // Set up pin modes
  pinMode(STEERING_PIN, INPUT);

  // Start Serial
  Serial.begin(57600);
}

void loop() {
  // Read in raw value
  int data = analogRead(STEERING_PIN);

  // Convert to ROS message type
  raw_data.data = data;

  // Publish message
  raw_pot_pub.publish(&raw_data);
  
  nh.spinOnce();
  delay(DELAY_TIME);
}
