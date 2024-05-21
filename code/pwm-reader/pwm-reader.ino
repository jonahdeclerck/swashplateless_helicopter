const int ch1Pin = 2;  
const int ch2Pin = 3;  
const int ch3Pin = 4;  
const int ch4Pin = 5;  

volatile unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4;
volatile unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw;

void setup() {
  Serial.begin(9600); 

  attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
}

void loop() {
  Serial.print("2500,");
  Serial.print("500,");

  Serial.print("Channel 1: ");
  Serial.print(channel_1_raw);
  Serial.print(",");

  Serial.print("Channel 2: ");
  Serial.print(channel_2_raw);
  Serial.print(",");

  Serial.print("Channel 3: ");
  Serial.print(channel_3_raw);
  Serial.print(",");
  
  Serial.print("Channel 4: ");
  Serial.println(channel_4_raw);

  delay(5);
}

void getCh1() {
  if(digitalRead(ch1Pin) == HIGH) {
    rising_edge_start_1 = micros();
  } else {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  if(digitalRead(ch2Pin) == HIGH) {
    rising_edge_start_2 = micros();
  } else {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  if(digitalRead(ch3Pin) == HIGH) {
    rising_edge_start_3 = micros();
  } else {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  if(digitalRead(ch4Pin) == HIGH) {
    rising_edge_start_4 = micros();
  } else {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}
