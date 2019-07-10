/**
 * This code allows you to write a signal to a relay to act as
 * the switch for the accelerator and to then send two signals
 * that mimic the potentiometers to control the forklift speed.
 * 
 * This code is intended to be used as with an Arduino as a 
 * serial node in ROS and receives the "pedal_switch" and 
 * "pedal_fraction" topic messages from the "accelerator_node"
 * node.
 * 
 * It also reads in the encoder from the steering stepper motor
 * determines whether the motor has stopped moving or not.
 * 
 * Pin Layout
 * Arduino        Pedal
 * D4->relay in   Relay NO->Switch (green), Relay COM->Switch GND (black)
 * D5             Signal 1 (blue)
 * D6             Signal 2 (red)
 * GND            GND (yellow)
 * 
 *                DAC
 * A4             SDA
 * A5             SCL
 * 
 *                Encoder
 * D2 (interrupt) Pin A (white)
 * D3             Pin B (Brown)
 * 5V             5V (Orange)
 * GND            GND (Black)
 */

//===== ROS Includes =====//
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <Wire.h>

//===== Function declarations =====//
void switchCallback(const std_msgs::Bool&);
void pedalCallback(const std_msgs::UInt8&);
void writeToAnalog(const int analog);

//===== I2C Connection =====//
const int MCP4725_ADDR = 0x60;
// The DAC is 12 bit
const int ANALOG_MAX = 4095;
const int ANALOG_MIN = 0;

//===== Pins =====//
const int ENCODER_PINA = 2;
const int ENCODER_PINB = 3;
const int RELAY_PIN = 4;
const int DEBUG_LED = 13;

//===== Pedal Variables =====//
// You want to "MIN" and "MAX" pwm values to produce voltages 
// that match the min and max of the signals from the forklift
const int PWM_MIN = 15;
const int PWM_MAX = 200;
bool pedal_switch;
uint8_t pedal_pwm;

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
ros::Subscriber<std_msgs::Bool> pedal_switch_sub("velocity_node/pedal_switch", &switchCallback);
ros::Subscriber<std_msgs::UInt8> pedal_fraction_sub("velocity_node/pedal_pwm", &pedalCallback);

void setup() {
  //--- Set up ROS
  nh.initNode();
  nh.subscribe(pedal_switch_sub);
  nh.subscribe(pedal_fraction_sub);
  nh.advertise(moving_pub);
    
  //--- Set up pin modes
  // Pedal
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  // Encoder
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);
  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), isr, RISING); // ISR = Interrupt Service Routine

  // Begin I2C connection to DAC
  Wire.begin();

  // Begin serial connection
  Serial.begin(57600);
}

void loop() {
  //===== Pedal Sequence =====//
  // Update switch relay
  if (pedal_switch) {
    digitalWrite(RELAY_PIN, LOW);
  }
  else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  // Update pedal signals
  int pwm_signal = min(pedal_pwm, PWM_MAX);
  pwm_signal = max(pwm_signal, PWM_MIN);
  // Convert to DAC board resolution
  /* NOTE: the current controls have been tuned to work with a PWM range of 0-255.
   * The Digital to Analog converted board has a resolution of 12 bits so it has a 
   * range of 0 to 4095. We are currently using the tuned controller for 0-255 
   * since the change between steps is larger and changing the range would then
   * necessitate retuning all of the control parameters. The forklift speed 
   * resolution is not high enough to make the change necessary, so we are leaving
   * the current tuned parameters as-is and are converting the 0-255 signal value
   * into the appropriate 0-4095 range and then sending it to the DAC.
   */
  int analog_signal = map(pwm_signal, 0, 255, 0, 4095);
  writeToAnalog(analog_signal);

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
  pedal_switch = msg.data;
}

void pedalCallback(const std_msgs::UInt8& msg)
{
  pedal_pwm = msg.data;
}

void writeToAnalog(const int analog)
{
  // Bound the input to the DAC
  int input;
  input = max(analog, ANALOG_MIN);
  input = min(input, ANALOG_MAX);

  // Send "write" signal to device
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64); // "write" command for DAC input register

  // Send data, most significant byte first
  byte MSB = (input >> 4);
  byte LSB = ((input & 15) << 4); // expects the last four bits in the msb position
  Wire.write(MSB);
  Wire.write(LSB);
  Wire.endTransmission();
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
