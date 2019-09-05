/*
 *  Node for reading steering angular speeds and controls the stepper motor  
 *  that spins the steering wheel.
 */
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//===== Pins =====//
const int STEP_COMMAND_PIN = 5;
const int DIRECTION_PIN = 6;
const int ENABLE_PIN = 4;

//===== ROS Ojbects =====//
ros::NodeHandle nh;
float velocity = 0.0; // desired velocity in steps/sec
bool enable = false; // turns the motor power on and off
void velocityCallback(std_msgs::Float32 &msg);
void enableCallback(std_msgs::Bool &msg);
ros::Subscriber<std_msgs::Float32> velocity_sub("/steering_node_A4988/motor/velocity", &velocityCallback);
ros::Subscriber<std_msgs::Bool> enable_sub("/steering_node_A4988/motor/enable", &enableCallback);
 
void setup() {
  //----- Set up ROS -----//
  nh.initNode();
  nh.subscribe(velocity_sub);
  nh.subscribe(enable_sub);
  Serial.begin(57600);

  //----- Set Pin modes -----//
  pinMode(STEP_COMMAND_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
}
void loop() {
  if (enable) {
    digitalWrite(ENABLE_PIN, LOW); // LOW turns the motors ON
    
    bool spin_positive = (velocity >= 0);

    if (spin_positive) {
      digitalWrite(DIRECTION_PIN, HIGH);
    }
    else {
      digitalWrite(DIRECTION_PIN, LOW);
    }

    // Get the delay time to maintain the desired velocity
    float delay_time = 1/fabs(velocity); // {sec/step}

    // Convert to microseconds
    delay_time = delay_time * 1000000;

    // Divide by 2 to get the pause between each digitalWrite
    delay_time = delay_time/2;

    // Step the motor
    delay_time = (unsigned int)delay_time; // convert to unsigned int for delayMicroseconds()
    digitalWrite(STEP_COMMAND_PIN, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(STEP_COMMAND_PIN, LOW);
    delayMicroseconds(delay_time);  
  }
  else {
    digitalWrite(ENABLE_PIN, HIGH); // HIGH turns the motors OFF
  }
  
  nh.spinOnce();
}

void velocityCallback(std_msgs::Float32 &msg)
{
  velocity = msg.data;
}

void enableCallback(std_msgs::Bool &msg)
{
  enable = msg.data;
}
