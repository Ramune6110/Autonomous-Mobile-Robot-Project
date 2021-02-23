/***********************************************************************
 * BMX055 with ROS
 **********************************************************************/
#include <ros.h>
#include <Wire.h>
#include <tf/tf.h>
#include "Arduino.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

/***********************************************************************
 * Macro Definition
 **********************************************************************/
#define Addr_Accl 0x19  //BMX055　加速度センサのI2Cアドレス  
#define Addr_Gyro 0x69  //BMX055　ジャイロセンサのI2Cアドレス
#define sampleFreqDef 50.0f   // sample frequency in Hz
#define betaDef       0.033f  // 2 * proportional gain

// encoderのA相, B相
#define left_outputA   32
#define left_outputB   33
#define right_outputA  34
#define right_outputB  35

// モータドライバの正転、逆転を制御するピン
#define EN_L 12
#define IN1_L 7
#define IN2_L 8

#define EN_R 11
#define IN1_R 5
#define IN2_R 6

/***********************************************************************
 * Global variables
 **********************************************************************/
// sensor value
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;

// sensor offset
float xAcclOffset = 0.18f;
float yAcclOffset = -0.27f;
float zAcclOffset = 0.0f;
float xGyroOffset = 0.0f;
float yGyroOffset = 0.0f;
float zGyroOffset = 0.0f;

// Euler angle
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// quaternion
float invSampleFreq;
float beta;        // algorithm gain
float q0;
float q1;
float q2;
float q3; // quaternion of sensor frame relative to auxiliary frame
char anglesComputed;

// left encoder
int left_counter = 0; 
int left_counter_A;
int left_preoutput_A;  

// right encoder
int right_counter = 0; 
int right_counter_A;
int right_preoutput_A;  

// cmd_vel
double w_r = 0.0;
double w_l = 0.0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.03;
double wheel_sep = 0.32;

double speed_ang = 0.0;
double speed_lin = 0.0;

/***********************************************************************
 * ROS Function
 **********************************************************************/
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

/**********************************************************************
 * ROS Parameter
 **********************************************************************/
ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pubimu("imu/data", &imu);

std_msgs::Float32 a;
std_msgs::Float32 b;
//Publish steps of left and right stepper motors
ros::Publisher pub1("lwheel", &a);
ros::Publisher pub2("rwheel", &b);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/***********************************************************************
 * Prototype Definition
 **********************************************************************/
void BMX055_Init();
void Madgwick_Init();
void BMX055_Accl();
void BMX055_Gyro();
void MadgwickAHRS_IMU(double xGyro, double yGyro, double zGyro, double xAccl, double yAccl, double zAccl);
float getPitch();
float getYaw();
float getRollRadians();
float getPitchRadians();
float getYawRadians();
float invSqrt(float x);
void computeAngles();

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

/**
 *****************************************************************************************************
 * setup
 *****************************************************************************************************
 */
void setup()
{
    Wire.begin();       // Wire(Arduino-I2C)の初期化
    Serial.begin(9600); // デバック用シリアル通信は9600bps

    BMX055_Init();      //BMX055 初期化
    Madgwick_Init();    //MadgwickFilter 初期化

    // モータのプラス、マイナスピン設定
    pinMode (left_outputA,INPUT);
    pinMode (left_outputB,INPUT);
    pinMode (right_outputA,INPUT);
    pinMode (right_outputB,INPUT);

    // モータドライバのピン設定
    Motors_init();
    // Reads the initial state of the left_outputA
    left_preoutput_A  = digitalRead(left_outputA);   
    right_preoutput_A = digitalRead(right_outputA);   

    // ROS Setup
    nh.initNode();
    nh.advertise(pubimu);
    nh.advertise(pub1);
    nh.advertise(pub2);
    nh.subscribe(sub);
    delay(300);
}

/**
 *****************************************************************************************************
 * main loop
 *****************************************************************************************************
 */
void loop()
{
    /**********************************************************************
    * Get IMU Data
    **********************************************************************/
    BMX055_Accl();

    BMX055_Gyro();

    // offset
    xAccl = xAccl + xAcclOffset;
    yAccl = yAccl + yAcclOffset;
    zAccl = zAccl + zAcclOffset;
    xGyro = xGyro + xGyroOffset;
    yGyro = yGyro + yGyroOffset;
    zGyro = zGyro + zGyroOffset;

    // madgwick filter
    MadgwickAHRS_IMU(xGyro, yGyro, zGyro, xAccl, yAccl, zAccl);

    roll  = getRoll();
    pitch = getPitch();
    yaw   = getYaw()-180.0;

    Serial.print("Roll= ");
    Serial.print(roll); 
    Serial.print(",");
    Serial.print("Pitch= ");
    Serial.print(pitch); 
    Serial.print(",");
    Serial.print("Yaw= ");
    Serial.print(yaw);
    Serial.println("");

    xGyro = xGyro * (PI / 180.0);
    yGyro = yGyro * (PI / 180.0);
    zGyro = zGyro * (PI / 180.0);
  
    /**********************************************************************
    * Moter Drive
    **********************************************************************/
    // モータに入力を加える
    MotorL(w_l * 30); 
    MotorR(w_r*  30);

    //MotorL(w_l * 25); 
    //MotorR(w_r*  25);

    // エンコーダのカウント
    update_left();
    update_right();

    // 左右のencoder値の符号が異なるので、前進を+, 後進を-で統一した
    a.data = left_counter;
    b.data = right_counter;

    /*********************************************************************
    * ROS Publish
    **********************************************************************/
    // imu publish
    imu.header.frame_id = "imu_link";
    imu.header.stamp = nh.now();
    imu.angular_velocity.x = xGyro;
    imu.angular_velocity.y = yGyro;
    imu.angular_velocity.z = zGyro; // [rad/sec]
    imu.linear_acceleration.x = xAccl;      
    imu.linear_acceleration.y = yAccl;  
    imu.linear_acceleration.z = zAccl; 

    imu.orientation.w = q0;
    imu.orientation.x = q1;
    imu.orientation.y = q2;
    imu.orientation.z = q3;
    pubimu.publish(&imu);
    
    // encoder publish
    pub1.publish(&a);
    pub2.publish(&b);

    nh.spinOnce();

    delay(10);
}

/**********************************************************************
 * BMX055 Configuration
 **********************************************************************/
void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
}
//=====================================================================================//
void BMX055_Accl()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098+0.5; // renge +-2g
  yAccl = yAccl * 0.0098+0.5; // renge +-2g
  zAccl = zAccl * 0.0098-0.3; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s LSB sensitivity: 262.4 LSB/dps 
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s LSB sensitivity: 262.4 LSB/dps 
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s LSB sensitivity: 262.4 LSB/dps 
}

/**********************************************************************
 * Magwick filter
 **********************************************************************/
void Madgwick_Init() {
  beta = betaDef;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  invSampleFreq = 1.0f / sampleFreqDef;
  anglesComputed = 0;
}
//=====================================================================================//
void MadgwickAHRS_IMU(double gx, double gy, double gz, double ax, double ay, double az) {
   double recipNorm;
   double s0, s1, s2, s3;
   double qDot1, qDot2, qDot3, qDot4;
   double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

   // Convert gyroscope degrees/sec to radians/sec
   gx *= 0.0174533;
   gy *= 0.0174533;
   gz *= 0.0174533;

   // Rate of change of quaternion from gyroscope
   qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
   qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
   qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
   qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
   if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

     // Normalise accelerometer measurement
     recipNorm = invSqrt(ax * ax + ay * ay + az * az);
     ax *= recipNorm;
     ay *= recipNorm;
     az *= recipNorm;

     // Auxiliary variables to avoid repeated arithmetic
     _2q0 = 2.0 * q0;
     _2q1 = 2.0 * q1;
     _2q2 = 2.0 * q2;
     _2q3 = 2.0 * q3;
     _4q0 = 4.0 * q0;
     _4q1 = 4.0 * q1;
     _4q2 = 4.0 * q2;
     _8q1 = 8.0 * q1;
     _8q2 = 8.0 * q2;
     q0q0 = q0 * q0;
     q1q1 = q1 * q1;
     q2q2 = q2 * q2;
     q3q3 = q3 * q3;

     // Gradient decent algorithm corrective step
     s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
     s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
     s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
     s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
     recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
     s0 *= recipNorm;
     s1 *= recipNorm;
     s2 *= recipNorm;
     s3 *= recipNorm;

     // Apply feedback step
     qDot1 -= beta * s0;
     qDot2 -= beta * s1;
     qDot3 -= beta * s2;
     qDot4 -= beta * s3;
   }

   // Integrate rate of change of quaternion to yield quaternion
   q0 += qDot1 * invSampleFreq;
   q1 += qDot2 * invSampleFreq;
   q2 += qDot3 * invSampleFreq;
   q3 += qDot4 * invSampleFreq;

   // Normalise quaternion
   recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   q0 *= recipNorm;
   q1 *= recipNorm;
   q2 *= recipNorm;
   q3 *= recipNorm;
   anglesComputed = 0.0;
 }
 
//=====================================================================================//
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}
//=====================================================================================//
float getRoll() {
    if (!anglesComputed) computeAngles();
    return roll * 57.29578f;
}
float getPitch() {
    if (!anglesComputed) computeAngles();
    return pitch * 57.29578f;
}
float getYaw() {
    if (!anglesComputed) computeAngles();
    return yaw * 57.29578f + 180.0f;
}
float getRollRadians() {
    if (!anglesComputed) computeAngles();
    return roll;
}
float getPitchRadians() {
    if (!anglesComputed) computeAngles();
    return pitch;
}
float getYawRadians() {
    if (!anglesComputed) computeAngles();
    return yaw;
}
//=====================================================================================//
void computeAngles()
{
  roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
  pitch = asinf(-2.0f * (q1*q3 - q0*q2));
  yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
  anglesComputed = 1;
}
//=====================================================================================//

/***********************************************************************
 * Moter Function
 **********************************************************************/
void Motors_init(){

 pinMode(EN_L, OUTPUT);

 pinMode(EN_R, OUTPUT);

 pinMode(IN1_L, OUTPUT);

 pinMode(IN2_L, OUTPUT);

 pinMode(IN1_R, OUTPUT);

 pinMode(IN2_R, OUTPUT);

 digitalWrite(EN_L, LOW);

 digitalWrite(EN_R, LOW);

 digitalWrite(IN1_L, LOW);

 digitalWrite(IN2_L, LOW);

 digitalWrite(IN1_R, LOW);

 digitalWrite(IN2_R, LOW);

}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){

     if (Pulse_Width1 > 240) {
        Pulse_Width1 = 240;
     }
     
     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, LOW);

     digitalWrite(IN2_L, HIGH);

 }

 if (Pulse_Width1 < 0){

     Pulse_Width1=abs(Pulse_Width1);

     if (Pulse_Width1 > 240) {
        Pulse_Width1 = 240;
     }

     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, HIGH);

     digitalWrite(IN2_L, LOW);

 }

 if (Pulse_Width1 == 0){

     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, LOW);

     digitalWrite(IN2_L, LOW);

 }

}


void MotorR(int Pulse_Width2){


 if (Pulse_Width2 > 0){

     if (Pulse_Width2 > 240) {
        Pulse_Width2 = 240;
     }

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, LOW);

     digitalWrite(IN2_R, HIGH);

 }

 if (Pulse_Width2 < 0){

     Pulse_Width2=abs(Pulse_Width2);

     if (Pulse_Width2 > 240) {
        Pulse_Width2 = 240;
     }

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, HIGH);

     digitalWrite(IN2_R, LOW);

 }

 if (Pulse_Width2 == 0){

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, LOW);

     digitalWrite(IN2_R, LOW);

 }

}

/***********************************************************************
 * Encoder Function
 **********************************************************************/
void update_left()
{
    left_counter_A = digitalRead(left_outputA); // Reads the "current" state of the back_left_outputA
    // If the previous and the current state of the back_left_outputA are different, that means a Pulse has occured
    if (left_counter_A != left_preoutput_A){     
        // If the back_left_outputB state is different to the back_left_outputA state, that means the encoder is rotating clockwise
        if (digitalRead(left_outputB) != left_counter_A) { 
        left_counter ++;
        } else {
        left_counter --;
        }
        Serial.print("left Position: ");
        Serial.println(left_counter);
    } 
    left_preoutput_A = left_counter_A; // Updates the previous state of the back_left_outputA with the current state
}

void update_right()
{
    right_counter_A = digitalRead(right_outputA); // Reads the "current" state of the back_left_outputA
    // If the previous and the current state of the back_left_outputA are different, that means a Pulse has occured
    if (right_counter_A != right_preoutput_A){     
        // If the back_left_outputB state is different to the back_left_outputA state, that means the encoder is rotating clockwise
        if (digitalRead(right_outputB) != right_counter_A) { 
        right_counter ++;
        } else {
        right_counter --;
        }
        Serial.print("right Position: ");
        Serial.println(right_counter);
    } 
    right_preoutput_A = right_counter_A; // Updates the previous state of the back_left_outputA with the current state
}