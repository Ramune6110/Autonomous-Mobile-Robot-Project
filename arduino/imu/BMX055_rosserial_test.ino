/***********************************************************************
 * BMX055 with ROS
 **********************************************************************/
#include <ros.h>
#include <Wire.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

/***********************************************************************
 * Macro Definition
 **********************************************************************/
// BMX055　加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)
#define sampleFreqDef   50.0f          // sample frequency in Hz
#define betaDef         0.033f           // 2 * proportional gain

/***********************************************************************
 * Global variables
 **********************************************************************/
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;

float xAcclOffset = 0.18f;
float yAcclOffset = -0.27f;
float zAcclOffset = 0.0f;
float xGyroOffset = 0.0f;
float yGyroOffset = 0.0f;
float zGyroOffset = 0.0f;
int xMagOffset    = 50;
int yMagOffset    = 50;
int zMagOffset    = 50;

float roll=0.00;
float pitch=0.00;
float yaw=0.00;

float invSampleFreq;
float beta;        // algorithm gain
float q0;
float q1;
float q2;
float q3; // quaternion of sensor frame relative to auxiliary frame

char anglesComputed;

/**********************************************************************
 * ROS Parameter
 **********************************************************************/
ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pubimu("imu/data", &imu);

/***********************************************************************
 * Prototype Definition
 **********************************************************************/
void BMX055_Init();
void Madgwick_Init();
void BMX055_Accl();
void BMX055_Gyro();
void BMX055_Mag();
void MadgwickAHRS_IMU(double xGyro, double yGyro, double zGyro, double xAccl, double yAccl, double zAccl);
void MadgwickAHRS(double xGyro, double yGyro, double zGyro, double xAccl, double yAccl, double zAccl, double xMag, double yMag, double zMag);
void updateIMU(float xGyro, float yGyro, float zGyro, float xAccl, float yAccl, float zAccl);
float getRoll();
float getPitch();
float getYaw();
float getRollRadians();
float getPitchRadians();
float getYawRadians();
float invSqrt(float x);
void computeAngles();

/**********************************************************************
 * Setup
 **********************************************************************/
void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバック用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  
  //MadgwickFilter 初期化
  Madgwick_Init();

  nh.initNode();
  nh.advertise(pubimu);
  
  delay(300);
}

/**********************************************************************
 * loop
 **********************************************************************/
void loop()
{
   /**********************************************************************
   * Get IMU Data
   **********************************************************************/
   BMX055_Accl();
  
   BMX055_Gyro();
  
   BMX055_Mag();

   // offset
   xAccl = xAccl + xAcclOffset;
   yAccl = yAccl + yAcclOffset;
   zAccl = zAccl + zAcclOffset;
   xGyro = xGyro + xGyroOffset;
   yGyro = yGyro + yGyroOffset;
   zGyro = zGyro + zGyroOffset;
   xMag  = xMag + xMagOffset;
   yMag  = yMag + yMagOffset;
   zMag  = zMag + zMagOffset; 

   // madgwick filter
   MadgwickAHRS_IMU(xGyro, yGyro, zGyro, xAccl, yAccl, zAccl);
   //MadgwickAHRS(xGyro, yGyro, zGyro, xAccl, yAccl, zAccl, xMag, yMag, zMag);
   
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

  /*********************************************************************
  * ROS Publish
  **********************************************************************/
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
  
  /*mag.header.frame_id = "imu_link";
  mag.header.stamp = nh.now();
  mag.magnetic_field.x = xMag;
  mag.magnetic_field.y = yMag;
  mag.magnetic_field.z = zMag; // [μT]
  pubmag.publish(&mag);*/

  /*tfs_msg.header.stamp = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = q0;
  tfs_msg.transform.rotation.x = q1;
  tfs_msg.transform.rotation.y = q2;
  tfs_msg.transform.rotation.z = q3;
  tfbroadcaster.sendTransform(tfs_msg);*/
  
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
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
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
//=====================================================================================//
void BMX055_Mag()
{
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] <<8) | (data[0]>>3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] <<8) | (data[2]>>3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] <<8) | (data[4]>>3));
  if (zMag > 16383)  zMag -= 32768;
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
 
void MadgwickAHRS(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) {
 double recipNorm;
 double s0, s1, s2, s3;
 double qDot1, qDot2, qDot3, qDot4;
 double hx, hy;
 double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

 // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
 if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
   MadgwickAHRS_IMU(gx, gy, gz, ax, ay, az);
 }

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

   // Normalise magnetometer measurement
   recipNorm = invSqrt(mx * mx + my * my + mz * mz);
   mx *= recipNorm;
   my *= recipNorm;
   mz *= recipNorm;

   // Auxiliary variables to avoid repeated arithmetic
   _2q0mx = 2.0 * q0 * mx;
   _2q0my = 2.0 * q0 * my;
   _2q0mz = 2.0 * q0 * mz;
   _2q1mx = 2.0 * q1 * mx;
   _2q0 = 2.0 * q0;
   _2q1 = 2.0 * q1;
   _2q2 = 2.0 * q2;
   _2q3 = 2.0 * q3;
   _2q0q2 = 2.0 * q0 * q2;
   _2q2q3 = 2.0 * q2 * q3;
   q0q0 = q0 * q0;
   q0q1 = q0 * q1;
   q0q2 = q0 * q2;
   q0q3 = q0 * q3;
   q1q1 = q1 * q1;
   q1q2 = q1 * q2;
   q1q3 = q1 * q3;
   q2q2 = q2 * q2;
   q2q3 = q2 * q3;
   q3q3 = q3 * q3;

   // Reference direction of Earth's magnetic field
   hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
   hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
   _2bx = sqrtf(hx * hx + hy * hy);
   _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
   _4bx = 2.0 * _2bx;
   _4bz = 2.0 * _2bz;

   // Gradient decent algorithm corrective step
   s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
   s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
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

    roll  = getRoll();
    pitch = getPitch();
    yaw   = getYaw();
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