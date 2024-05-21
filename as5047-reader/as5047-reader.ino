// include the library for the AS5047P sensor.
#include <AS5047P.h>
#include <SPI.h>

// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT 10

// define the spi bus speed 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

//GREEN - CS - 10
//YELLOW - MOSI - 11
//WHITE - MISO - 12
//BLUE - SCK - 14 (remapped from default 13 to keep onboard LED working)


// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

void setup() {
  SPI.setSCK(14); //Remap SCK to pin 14 to keep onboard LED working

  // initialize the serial bus for the communication with your pc.
  Serial.begin(115200);


  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

}

void loop() {


  Serial.print("0,");
  Serial.print("360,");
  Serial.print("as5047 angle: ");
  Serial.println(as5047p.readAngleDegree() * PI/180);      // read the angle value from the AS5047P sensor an print it to the serial consol.

  delay(10);                                     // wait for 500 milli seconds.

}