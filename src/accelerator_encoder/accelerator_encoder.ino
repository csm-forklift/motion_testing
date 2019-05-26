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
 * It also reads in the encoder from the steering stepper motor
 * determines whether the motor has stopped moving or not.
 * 
 * Pin Layout
 * Arduino        Pedal
 * D4->relay in   NO->Switch (green), COM->Switch GND (black)
 * D5             Signal 1 (blue)
 * D6             Signal 2 (red)
 * GND            GND (yellow)
 * 
 *                Encoder
 * D2 (interrupt) Pin A (white)
 * D3             Pin B (Brown)
 * 5V             5V (Orange)
 * GND            GND (Black)
 */

// ROS Includes
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

// Function declarations
void switchCallback(const std_msgs::Bool&);
void pedalCallback(const std_msgs::Int8&);

//===== Pins =====//
const int ENCODER_PINA = 2;
const int ENCODER_PINB = 3;
const int RELAY_PIN = 4;
const int SIGNAL_PIN_1 = 5;
const int SIGNAL_PIN_2 = 6;
const int DEBUG_LED = 13;

//===== Pedal Variables =====//
// You want to "MIN" and "MAX" pwm values to produce voltages 
// that match the min and max of the signals from the forklift
const int PWM_MIN = 15;  
const int PWM_MAX = 200; 
bool throttle_switch;
int8_t pedal_pwm;

//===== Encoder Variables =====//
volatile int counter = 0;
int prev_counter = 0;
int repeats = 0;
bool moving = false; // indicates whether the motor is moving or not

//===== Parameters =====//
const int DELAY_TIME = 32; // ms
// Number of times the counter value must remain the same 
// before considering the motor to be stopped.
const int NUM_REPEATS = 3;

//===== ROS Objects =====//
ros::NodeHandle nh;
std_msgs::Bool is_moving;
ros::Publisher moving_pub("steering_node/motor/is_moving", &is_moving);
ros::Subscriber<std_msgs::Bool> throttle_switch_sub("accelerator_node/throttle_switch", &switchCallback);
ros::Subscriber<std_msgs::Int8> pedal_fraction_sub("accelerator_node/pedal_pwm", &pedalCallback);

void setup() {
  //--- Set up ROS
  nh.initNode();
  nh.subscribe(throttle_switch_sub);
  nh.subscribe(pedal_fraction_sub);
    
  //--- Set up pin modes
  // Pedal
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SIGNAL_PIN_1, OUTPUT);
  pinMode(SIGNAL_PIN_2, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  analogWrite(SIGNAL_PIN_1, PWM_MIN);
  analogWrite(SIGNAL_PIN_2, PWM_MIN);
  // Encoder
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);
  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), isr, RISING); // ISR = Interrupt Service Routine

  // Begin serial connection
  Serial.begin(57600);
}

void loop() {
  //===== Pedal Sequence =====//
  // Update switch relay
  if (throttle_switch) {
    digitalWrite(RELAY_PIN, LOW);
  }
  else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  // Update pedal signals
  int pwm_signal = min(pedal_pwm, PWM_MAX);
  pwm_signal = max(pwm_signal, PWM_MIN);
  analogWrite(SIGNAL_PIN_1, pwm_signal);
  analogWrite(SIGNAL_PIN_2, pwm_signal);

  //===== Encoder Sequence =====//
  // Check if counter is the same as the previous value
  if (counter == prev_counter) {
    repeats++;
    if (repeats > NUM_REPEATS) {
      moving = false;
    }
    else {
      moving = true;
    }
  }
  else {
    repeats = 0;
    prev_counter = counter;
    moving = true;
  }

  // Publish message
  is_moving.data = moving;
  moving_pub.publish(&is_moving);

  nh.spinOnce();
  delay(DELAY_TIME);
}

void switchCallback(const std_msgs::Bool& msg)
{
  throttle_switch = msg.data;
}

void pedalCallback(const std_msgs::Int8& msg)
{
  pedal_pwm = msg.data;
}

void isr() {
  int channel_A = digitalRead(ENCODER_PINA);
  int channel_B = digitalRead(ENCODER_PINB);

  if (channel_A == channel_B) {
    counter ++;
  }
  else {
    counter--;
  }
}
