#include <Servo.h>

Servo esc;  // Create a servo object

void setup() {
  esc.attach(9);  // Attach the ESC signal wire to pin 9
  esc.writeMicroseconds(1500);  // Send a neutral signal (1500 microseconds)
  delay(1000);  // Delay for ESC initialization
}

void loop() {
  // Change the throttle position
  // 1000 microseconds = full reverse, 1500 microseconds = neutral, 2000 microseconds = full forward
  esc.writeMicroseconds(1500);  // Set throttle to neutral (1500 microseconds)
  delay(1000);  // Delay for 1 second
  
  esc.writeMicroseconds(1000);  // Set throttle to full reverse (1000 microseconds)
  delay(1000);  // Delay for 1 second
  
  esc.writeMicroseconds(2000);  // Set throttle to full forward (2000 microseconds)
  delay(1000);  // Delay for 1 second
}
