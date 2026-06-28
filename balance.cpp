#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#define pitch A4
#define roll A5
Servo left;
Servo right;
// rudder vals
const int low_bound = 40;
const int up_bound = 140;
// Define these globally at the top of your code
float Kp = 1.5; 
float Ki = 0.05;
Adafruit_MPU6050 mpu;
int l = 90;
int r = 90;
// Global variables for tracking integrals
float integralR;
float integralP;
// Sensors data structures
sensors_event_t accel, gyro, temp;
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(pitch, INPUT);
  pinMode(roll, INPUT);
  Serial.begin(9600);
  left.attach(12);
  right.attach(11);
  if (!mpu.begin()) {
    Serial.print("failed to recognise");
    while (1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // Fetch new data from the sensor first
  mpu.getEvent(&accel, &gyro, &temp);
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  // Calculate pitch and roll in degrees
  float p = (atan2(ax, az) * 180 / PI);
  float ro = (atan2(ay, az) * 180 / PI);
  
  // check values first
  Serial.print(ro); // Changed from 'r' to 'ro' to correctly print roll value
  Serial.print(" = roll | pitch = ");
  Serial.println(p);
  
  // Integral calculations
  integralP -= p;
  integralR -= ro;
  
  // Constrain integrals to prevent windup
  integralP = constrain(integralP, -40, 40);
  integralR = constrain(integralR, -40, 40);
  l= 90+(p * Kp) + (integralP * Ki);
  r= 90+(ro * Kp) + (integralR * Ki);
  // Constrain servo targets to safety bounds
  l = constrain(l, 40, 140);
  r = constrain(r, 40, 140);
  // Print output values
  Serial.print(l);
  Serial.print(" = l | r = ");
  Serial.println(r);
  // Update servos
  left.write(l);
  right.write(r);
  delay(20);
}