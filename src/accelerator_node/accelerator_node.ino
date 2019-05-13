/**
 * This code logs analog values read from the analog pins on the
 * arduino and then prints them through Serial communication for
 * either directly printing to the Serial Monitor or to be 
 * intercepted by a Python program and stored in a '.csv' file.
 */

//const int NUM_PINS = 6;
//int PINS[NUM_PINS] = {A0, A1, A2, A3, A4, A5};
const int NUM_PINS = 1;
int PINS[NUM_PINS] = {A0};

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
  delay(50);
}
