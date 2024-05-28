//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//RX
#define USE_SBUS_RX

//IMU
#define USE_MPU6050_I2C 

//full scale gyro range (deg/sec)
#define GYRO_250DPS 

//full scale accelerometer range (G's)
#define ACCEL_2G 


//========================================================================================================================//

//libraries
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <AS5047P.h>  //Encoder library

#include "src/SBUS/SBUS.h"  //SBUS 
#include "src/MPU6050/MPU6050.h" //IMU
MPU6050 mpu6050;


//========================================================================================================================//

//Setup AS5047P encoder

//GREEN - CS - 10
//YELLOW - MOSI - 11
//WHITE - MISO - 12
//BLUE - SCK - 13

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);


//========================================================================================================================//

//Setup gyro and accel full scale value selection and scale factor

#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0

#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0


//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)


//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//Radio:
const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
const int ch6Pin = 22; //aux1 (free aux channel)
const int PPM_Pin = 23;

//OneShot125 ESC pin outputs:

const int m2Pin = 1;

//PWM servo or ESC outputs:
const int servo1Pin = 0;

PWMServo servo1;  //Create servo objects to control a servo or ESC with PWM


//========================================================================================================================//

//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

//AS5047P encoder
float motorRads;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;

// float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float s1_command_scaled, m2_command_scaled;
int s1_command_PWM, m2_command_PWM;

//Flight status
bool armedFly = false;


//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(m2Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM valu
  delay(5);

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Initialize AS5047P encoder
  AS5047Pinit();
  delay(5);

  //Initialize IMU communication
  IMUinit();
  delay(5);

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  delay(5);

  //Arm OneShot125 motors
  m2_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
  armMotors(); //Loop over commandMotors() until ESCs happily arm
}


//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//

void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;


  //-----------------debugging-----------------//

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printAS5047PData();   //Prints magnetic encoder data 
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  //printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

  // Get arming status
  armedStatus(); //Check if the throttle cut is off and throttle is low.

  //get motorRads
  getMotorRads();

  //Get vehicle state
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits
  
  //PID Controller - SELECT ONE:
  controlANGLE(); //Stabilize on angle setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); //Writes PWM value to servo object

  //Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}


//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration

  s1_command_scaled = thro_des + abs(pitch_PID) + abs(roll_PID) + (pitch_PID * cos(motorRads)) + (roll_PID *sin(motorRads));
  m2_command_scaled = (thro_des/10)+(0.5 + yaw_PID); // centered yaw control, plus a fraction of thro_des to counteract the main rotor rotation
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050) && (channel_2_pwm > 1050)) {
    armedFly = true;
  }
}

void AS5047Pinit() {
  //DESCRIPTION: Initialize AS5047P encoder
  while (!as5047p.initSPI())
  {
    Serial.println("AS5047P initialization unsuccessful");
    Serial.println("Check AS5047P wiring or try cycling power");
    while (1) {}
  }
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU

  Wire.begin();
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
  
  mpu6050.initialize();
  
  if (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  //From the reset state all registers should be 0x00, so we should be at
  //max sample rate with digital low pass filter(s) off.  All we need to
  //do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

void getMotorRads() {
    // DESCRIPTION: Get motor angle for mixer, range 0 to 1
    float angle = as5047p.readAngleDegree(); // Read the angle in degrees
    if (angle <= 180) {
        motorRads = angle / 180.0; // Map 0 to 180 degrees to 0 to 1
    } else {
        motorRads = (360 - angle) / 180.0; // Map 180 to 360 degrees to 1 to 0
    }
}


void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data

  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  mpu6050.getMotion6(&AcX, &AcZ, &AcY, &GyX, &GyZ, &GyY);

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = -AcY / ACCEL_SCALE_FACTOR; //Invert Y axis to match modified IMU orientation
  AccZ = AcZ / ACCEL_SCALE_FACTOR;

  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;

  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = -GyY / GYRO_SCALE_FACTOR; //Invert Y axis to match modified IMU orientation
  GyroZ = GyZ / GYRO_SCALE_FACTOR;

  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;

  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Compute the rate of change of the quaternion based on gyroscope data.
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  //Ensure accelerometer data is valid and normalize it to a unit vector.
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    //Calculate auxiliary variables to simplify the gradient descent algorithm.
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    //Calculate the corrective step using the gradient descent algorithm.
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Adjust the quaternion rate of change based on the feedback from the accelerometer.
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion, integration over time
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Convert the quaternion to Euler angles (roll, pitch, yaw) for easier interpretation.
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values

  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol

  //Scaled to 125us - 250us for oneshot125 protocol
  m2_command_PWM = m2_command_scaled*125 + 125;

  //Constrain commands to motors within oneshot125 bounds
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;

  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio

  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
  {
    //sBus scaling below is for Taranis-Plus and X4R-SB
    float scale = 0.615;  
    float bias  = 895.0; 
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias; 
  }

  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values

  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pin, oneshot125 protocol

  int wentLow = 0;
  int pulseStart, timer;
  int flagM2 = 0;

  
  //Write motor pin high
  digitalWrite(m2Pin, HIGH);

  pulseStart = micros();

  //Write motor pin low as correct pulse length is reached
  while (wentLow < 1 ) { 
    timer = micros();
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
  }
}

void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()

  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered

  if ((channel_5_pwm < 1500) || (armedFly == false)) {
    armedFly = false;

    m2_command_PWM = 184;
    s1_command_PWM = 0;
  }
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz for consistency

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}


//========================================================================================================================//
//                                                      Debug                                                             //                           
//========================================================================================================================//

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print("Min:");
    Serial.print(1000);
    Serial.print(",Max:");
    Serial.print(2000);

    Serial.print(",CH1:");
    Serial.print(channel_1_pwm);
    Serial.print(",CH2:");
    Serial.print(channel_2_pwm);
    Serial.print(",CH3:");
    Serial.print(channel_3_pwm);
    Serial.print(",CH4:");
    Serial.print(channel_4_pwm);
    Serial.print(",CH5:");
    Serial.print(channel_5_pwm);
    Serial.print(",CH6:");
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printAS5047PData(){
  if (current_time - print_counter > 500) {
    print_counter = micros();
    
    Serial.print("Min:");
    Serial.print(0);
    Serial.print(",Max:");
    Serial.print(1);

    Serial.print(",MotorRads:");
    Serial.println(motorRads);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print("Min:");
    Serial.print(-250);
    Serial.print(",Max:");
    Serial.print(250);

    Serial.print((",GyroX:"));
    Serial.print(GyroX);
    Serial.print((",GyroY:"));
    Serial.print(GyroY);
    Serial.print((",GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print("Min:");
    Serial.print(-1);
    Serial.print(",Max:");
    Serial.print(1);

    Serial.print(F(",roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(",pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(",yaw_PID:"));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10) {
    print_counter = micros();

    Serial.print("Min:");
    Serial.print(0);
    Serial.print(",Max:");
    Serial.print(250);

    Serial.print(F(" m1_command:"));
    Serial.print(s1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.println(m2_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}


//========================================================================================================================//
//                                                  HELPER FUNCTIONS                                                      //                           
//========================================================================================================================//

float invSqrt(float x) {
  return 1.0/sqrtf(x); 
}