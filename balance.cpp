#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
Servo left;
Servo right;
// rudder vals
const int low_bound = 40;
const int up_bound = 140;
// Define these globally at the top of your code
float Kp = 2.0; 
float Ki = 0.0;
float Kd = 0.2;
Adafruit_MPU6050 mpu;
const float dt=.02;
float l = 90;
float r = 90;
float p=0;
float ro=0;
// Global variables for tracking integrals
float integralR=0;
float integralP=0;
// Sensors data structures
sensors_event_t accel, gyro, temp;
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  left.attach(12);
  right.attach(11);
  if (!mpu.begin()) {
    Serial.print("failed to recognise");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}
void loop() {
  // put your main code here, to run repeatedly:
  // Fetch new data from the sensor first
  mpu.getEvent(&accel, &gyro, &temp);
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float accelPitch = atan2(ax, az) * 180.0 / PI;
  float accelRoll  = atan2(ay, az) * 180.0 / PI;
  float dpitch = gyro.gyro.x * 180.0 / PI;
  float droll  = gyro.gyro.y * 180.0 / PI;
  // Calculate pitch and roll in degrees
  p = 0.98 * (p + dpitch * dt) + .02 * accelPitch;
  ro = 0.98 * (ro + droll * dt) + .02 * accelRoll;
  
  // check values first
  Serial.print(ro); // Changed from 'r' to 'ro' to correctly print roll value
  Serial.print(" = roll | pitch = ");
  Serial.println(p);
  if (abs(p) < 0.5) p = 0;
  if (abs(ro) < 0.5) ro = 0;
  
  // Integral calculations
  integralP += p*dt;
  integralR += ro*dt;
  
  // Constrain integrals to prevent windup
  integralP = constrain(integralP, -40, 40);
  integralR = constrain(integralR, -40, 40);
  l= 90+(p * Kp) + (integralP * Ki)-(dpitch*Kd);
  r= 90+(ro * Kp) + (integralR * Ki)-(droll*Kd);
  // Constrain servo targets to safety bounds
  l = constrain(l, low_bound, up_bound);
  r = constrain(r, low_bound, up_bound);
  // Print output values
  Serial.print(l); Serial.print(" = l | r = "); Serial.println(r);
  Serial.print("Pitch: "); Serial.print(p); Serial.print(" | Roll: "); Serial.println(ro);
  // Update servos
  left.write(180-(int)l);
  right.write(180-(int)r);
  delay((int)(dt*1000));
}
