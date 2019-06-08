/* Pin layout
 *  Arduino ---> Clamp control
 *  Red ---> GND  
 *  White ---> 5V  
 *  Black ---> Signal 1 
 *  Green ---> Signal 2 
 *  
 *  Limit switch ---> Arduino
 *  COM ---> 5V,  R-7
 *  NC ---> GND
 *  
 *  FSR ---> Arduino 
 *  FSR_R ---> 5V
 *  FSR_L ---> R-GND,A0 
 *  
 *  Stretch sensor ---> Arduino
 *  SS_r ---> 5V
 *  SS_l ---> 10k R -- GND, A1
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

std_msgs::Int16 force_msg;
std_msgs::Bool switch_msg;
std_msgs::Float32 stretch_msg;

void switchCallback(const std_msgs::Bool&);
void clampmovementCallback(const std_msgs::Float32&);
void clampgraspCallback(const std_msgs::Float32&);

// Limit switch
int limit_switch = 7;
const int led = 13;
bool switch_status;

//FSR
int fsrPin = 0;
int fsrReading;

// Stretch sensor
int stretch_sensor_pin = 1;

// Clamp switch
const int SIGNAL_PIN_1 = 3;
const int SIGNAL_PIN_2 = 4;
const int SIGNAL_PIN_3 = 5;
const int SIGNAL_PIN_4 = 6;
const int PWM_MIN = 0;
const int PWM_MIDDLE = 127;
const int PWM_MAX = 255;
bool clamp_switch;
float clamp_movement;
float clamp_grasp;


// ROS
ros::NodeHandle nh;

// Limit switch
ros::Publisher limit_switch_pub("switch_status", &switch_msg);

// FSR
ros::Publisher force_pub("force", &force_msg);

// Stretch sensor
ros::Publisher stretch_sensor_pub("stretch_length", &stretch_msg);

// Clamp switch 
ros::Subscriber<std_msgs::Bool> clamp_switch_sub("clamp_switch_node/clamp_switch", &switchCallback);
ros::Subscriber<std_msgs::Float32> clamp_movement_sub("clamp_switch_node/clamp_movement", &clampmovementCallback);
ros::Subscriber<std_msgs::Float32> clamp_grasp_sub("clamp_switch_node/clamp_grasp", &clampgraspCallback);
//
void setup() 
{
  Serial.begin(9600);
  nh.initNode();
  
  // Limit switch
  pinMode(limit_switch, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  nh.advertise(limit_switch_pub);
  
  // FSR
  nh.advertise(force_pub);

  // Stretch sensor
  nh.advertise(stretch_sensor_pub);
  
  // Clamp
  nh.subscribe(clamp_switch_sub);
  nh.subscribe(clamp_movement_sub);
  nh.subscribe(clamp_grasp_sub);

  pinMode(SIGNAL_PIN_1, OUTPUT);
  pinMode(SIGNAL_PIN_2, OUTPUT);
  
  analogWrite(SIGNAL_PIN_1, PWM_MIDDLE);
  analogWrite(SIGNAL_PIN_2, PWM_MIDDLE);
  analogWrite(SIGNAL_PIN_3, PWM_MIDDLE);
  analogWrite(SIGNAL_PIN_4, PWM_MIDDLE);
}

void loop() 
{
  // Limit switch
  if (digitalRead(limit_switch) == LOW)
  {
    digitalWrite(led, HIGH);
    switch_status = true;
  }
  else
  {
    digitalWrite(led, LOW);
    switch_status = false;
  }
  switch_msg.data = switch_status;
  limit_switch_pub.publish(&switch_msg);
  
  // FSR
  fsrReading = analogRead(fsrPin);
  force_msg.data = fsrReading;
  force_pub.publish(&force_msg);

  // Stretch sensor
  int value;
  int v_in = 5;
  float v_out = 0;
  float r_1 = 10;
  float r_2 = 0;
  float val;

  // Conductive Rubber Calculations
  float buffer = 0;
  value = analogRead(stretch_sensor_pin);
  v_out = (5.0 / 1023.0) * value;
  buffer = (v_in / v_out) - 1;
  r_2 = r_1 / buffer;
  val = 1000 / r_2;
  stretch_msg.data = val;
  stretch_sensor_pub.publish( &stretch_msg );
  
  // Clamp switch control  
  if (clamp_movement != 0)
  {
    int pwm_signal_move_1 = map(100*clamp_movement, -100, 100, PWM_MIN, PWM_MAX);
    int pwm_signal_move_2 = map(100*clamp_movement, -100, 100, PWM_MAX, PWM_MIN);

    analogWrite(SIGNAL_PIN_1, pwm_signal_move_1);
    analogWrite(SIGNAL_PIN_2, pwm_signal_move_2);
  }
  else
  {
    analogWrite(SIGNAL_PIN_1, PWM_MIDDLE);
    analogWrite(SIGNAL_PIN_2, PWM_MIDDLE);
  }
    
  if (clamp_grasp != 0) 
  {
    int pwm_signal_grasp_1 = map(100*clamp_grasp, -100, 100, PWM_MIN, PWM_MAX);
    int pwm_signal_grasp_2 = map(100*clamp_grasp, -100, 100, PWM_MAX, PWM_MIN);
  
    analogWrite(SIGNAL_PIN_3, pwm_signal_grasp_1);
    analogWrite(SIGNAL_PIN_4, pwm_signal_grasp_2);
  }
  else
  {
    analogWrite(SIGNAL_PIN_3, PWM_MIDDLE);
    analogWrite(SIGNAL_PIN_4, PWM_MIDDLE);
  }
  
  nh.spinOnce();
  delay(1);
}

// Clamp switch
void switchCallback(const std_msgs::Bool& msg)
{
  clamp_switch = msg.data;
}

// Clamp movement
void clampmovementCallback(const std_msgs::Float32& msg)
{
  clamp_movement = msg.data;
}

// Clamp grasp
void clampgraspCallback(const std_msgs::Float32& msg)
{
  clamp_grasp = msg.data;
}
