/**
 * This code allows you to write a signal to a relay to act as
 * the switch for the accelerator and to then send two signals
 * that mimic the potentiometers to control the forklift speed.
 * 
 * This code is intended to be used as with an Arduino as a 
 * serial node in ROS and receives the "throttle_switch" and 
 * "pedal_fraction" topic messages from the "accelerator_node"
 * node.
 * 
 * Pin Layout
 * Arduino        Pedal
 * D2->relay in   NO->Switch (green), COM->Switch GND (black)
 * D3             Signal 1 (blue)
 * D4             Signal 2 (red)
 * GND            GND (yellow)
 */

// ROS Includes
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// Function declarations
void switchCallback(const std_msgs::Bool&);
void pedalCallback(const std_msgs::Float32&);

//===== Pins =====//
//const int RELAY_PIN = 2;
//const int SIGNAL_PIN_1 = 3;
//const int SIGNAL_PIN_2 = 4;
const int RELAY_PIN = 4;
const int SIGNAL_PIN_1 = 5;
const int SIGNAL_PIN_2 = 6;
const int DEBUG_LED = 13;

//===== Data Variables =====//
// You want to "MIN" and "MAX" pwm values to produce voltages 
// that match the min and max of the signals from the forklift
const int PWM_MIN = 15;  
const int PWM_MAX = 200; 
bool g_throttle_switch;
float g_pedal_fraction;

//===== ROS Objects =====//
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> throttle_switch_sub("accelerator_node/throttle_switch", &switchCallback);
ros::Subscriber<std_msgs::Float32> pedal_fraction_sub("accelerator_node/pedal_fraction", &pedalCallback);

void setup() {
  // Set up ROS
  nh.initNode();
  nh.subscribe(throttle_switch_sub);
  nh.subscribe(pedal_fraction_sub);
    
  // Set up pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SIGNAL_PIN_1, OUTPUT);
  pinMode(SIGNAL_PIN_2, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  analogWrite(SIGNAL_PIN_1, PWM_MIN);
  analogWrite(SIGNAL_PIN_2, PWM_MIN);
}

void loop() {
  // Update switch relay
  if (g_throttle_switch) {
    digitalWrite(RELAY_PIN, LOW);
  }
  else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  // Update pedal signals
  int pwm_signal = map(100*g_pedal_fraction, 0, 100, PWM_MIN, PWM_MAX);
  analogWrite(SIGNAL_PIN_1, pwm_signal);
  analogWrite(SIGNAL_PIN_2, pwm_signal);

  nh.spinOnce();
  delay(1);
}

void switchCallback(const std_msgs::Bool& msg)
{
  g_throttle_switch = msg.data;
}

void pedalCallback(const std_msgs::Float32& msg)
{
  g_pedal_fraction = msg.data;
}
