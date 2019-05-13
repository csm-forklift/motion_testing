 /**
 * This code reads in the signal from the steering potentiometer and 
 * prints the data to serial. A ROS node then reads the serial data and publishes
 * it as a ROS message.
 * 
 * Pin Layout
 * Arduino    Steering Potentiometer
 * A0   ->    Signal (Black)
 * GND  ->    GND (yellow)
 */

//===== Pins =====//
const int STEERING_PIN = A0;

//===== Parameters =====//
const int DELAY_TIME = 32; // msec

void setup() {
  // Set up pin modes
  pinMode(STEERING_PIN, INPUT);

  // Set the reference as external
  //analogReference(AR_EXTERNAL);

  // Start Serial
  Serial.begin(57600);
}

void loop() {
  // Read in raw value
  int data = analogRead(STEERING_PIN);

  // Publish message
  Serial.println(data);
  
  delay(DELAY_TIME);
}
