/**
 * This code logs analog values read from the analog pins on the
 * arduino and then prints them through Serial communication for
 * either directly printing to the Serial Monitor or to be
 * intercepted by a Python program and stored in a '.csv' file.
 *
 *
 * Relay switch - Arduino ---> Gear switch
  *  7 ---> COM_1
  *  5 ---> NC_1
  *  3 ---> NO_1
  *  4 ---> COM_2
  *  6 ---> NC_2
  *  8 ---> NO_2
 */

#include <ros.h>
#include <std_msgs/Int8.h>

//const int NUM_PINS = 6;
//int PINS[NUM_PINS] = {A0, A1, A2, A3, A4, A5};

const int NUM_PINS = 1;
int PINS[NUM_PINS] = {A0};

// Gear switch
std_msgs::Int8 gear_msg;
void gearCallback(const std_msgs::Int8&);

int gear;
const int PIN_3 = 3;
const int PIN_8 = 4;

// ros
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int8> gear_switch_sub("/velocity_node/gear", &gearCallback);

void setup() {
  // Set up pins
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(PINS[i], INPUT);
  }

  // Begin Serial connection
  Serial.begin(115200);
  Serial.println("Begin Data Collection");
  for (int i = 0; i < NUM_PINS; i++) {
    if (i < (NUM_PINS - 1)) {
      Serial.print("Pin " + String(i) + ",");
    }
    else {
      Serial.println("Pin " + String(i));
    }
  }
  // gear
  nh.initNode();

  pinMode(PIN_3, OUTPUT);
  pinMode(PIN_8, OUTPUT);

  delay(1000);
}

void loop() {
  // Read data from the analog pins
  for (int i = 0; i < NUM_PINS; i++) {
    if (i < (NUM_PINS - 1)) {
      Serial.print(String(analogRead(PINS[i])) + ",");
    }
    else {
      Serial.println(String(analogRead(PINS[i])));
    }
  }

  // Set rate by changing delay time
  // 50ms delay = 20Hz
  // 32ms delay = 30Hz
  // gear
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
  delay(50);
}

void gearCallback(const std_msgs::Int8& msg)
{
  gear = msg.data;
}
