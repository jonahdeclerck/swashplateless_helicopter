// Define the PID variables
float pitch_PID = 1.0;
float roll_PID = 0.0;

// Define the motorRads variable
float motorRads = 0.0;
bool increasing = true;

// Define the throttle desired variable
float thro_des = 0.5;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
}

void loop() {
  // Increment or decrement motorRads between 0 and 0.5
  if (increasing) {
    motorRads += 0.01;
    if (motorRads >= 0.5) {
      increasing = false;
    }
  } else {
    motorRads -= 0.01;
    if (motorRads <= 0.0) {
      increasing = true;
    }
  }
  
  // Calculate the desired value
  // float calculated_value = thro_des + abs(pitch_PID) + abs(roll_PID) + (pitch_PID * cos(motorRads)) + (roll_PID * sin(motorRads));
  float calculated_value = sin(motorRads);
  // Print the calculated value and motorRads
  Serial.print("angle:");
  Serial.print(motorRads);
  Serial.print(", output:");
  Serial.println(calculated_value);

  // Small delay to slow down the loop for readability
  delay(100);
}
