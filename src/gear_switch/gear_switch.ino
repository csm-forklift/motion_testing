/*
Relay switch - Arduino ---> Gear switch
 *  7 ---> COM_1
 *  5 ---> NC_1
 *  3 ---> NO_1
 *  4 ---> COM_2
 *  6 ---> NC_2
 *  8 ---> NO_2
 *  */

#include <ros.h>
#include <std_msgs/Int8.h>

std_msgs::Int8 gear_msg;

void gearCallback(const std_msgs::Int8&);

int gear;
const int PIN_3 = 3; 
const int PIN_8 = 4; 

// ros
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int8> gear_switch_sub("/velocity_node/gear", &gearCallback);

void setup() 
{
  Serial.begin(115200);
  nh.initNode();

  pinMode(PIN_3, OUTPUT);
  pinMode(PIN_8, OUTPUT);  
}

void loop() 
{
  if (gear == 1)
  {
    digitalWrite(PIN_3, HIGH);
    digitalWrite(PIN_8, LOW);
  }
  
  else if (gear  == 0)
  {
    digitalWrite(PIN_3, HIGH);
    digitalWrite(PIN_8, HIGH);
  }
  
  else if (gear == -1)
  {
    digitalWrite(PIN_3, LOW);
    digitalWrite(PIN_8, HIGH);
  }  
  
  nh.spinOnce();
  delay(1);
}

void gearCallback(const std_msgs::Int8& msg)
{
  gear = msg.data;
}
