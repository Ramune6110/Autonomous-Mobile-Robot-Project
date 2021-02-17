/*
 * ********************************************************************************
 * SYSTEM            | moter_encoder_rosserial_integration
 * cmd_velの値をROSからsubscribeして、左右の回転速度に変換した値をモータに駆動して、
 * エンコーダの値をROSにpublishする機能を実装
 * 
 * 左右のencoder値の符号が逆 : 何かを勘違いしている？→辻褄合わせるだけなら符号を統一すれば良い
 * Input Output 1024bytesが良い
 * ********************************************************************************
*/
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "Arduino.h"

/***********************************************************************
 * Define
 **********************************************************************/
// モータのプラス、マイナスピン
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

/***********************************************************************
 * ROS Parameter
 **********************************************************************/
ros::NodeHandle nh;

std_msgs::Float32 a;
std_msgs::Float32 b;
//Publish steps of left and right stepper motors
ros::Publisher pub1("lwheel", &a);
ros::Publisher pub2("rwheel", &b);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/***********************************************************************
 * Prototype Function
 **********************************************************************/
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

/**
 *****************************************************************************************************
 * setup
 *****************************************************************************************************
 */
void setup() { 
    // モータのプラス、マイナスピン設定
    pinMode (left_outputA,INPUT);
    pinMode (left_outputB,INPUT);
    pinMode (right_outputA,INPUT);
    pinMode (right_outputB,INPUT);

    // モータドライバのピン設定
    Motors_init();
    
    Serial.begin (9600);
    // Reads the initial state of the left_outputA
    left_preoutput_A  = digitalRead(left_outputA);   
    right_preoutput_A = digitalRead(right_outputA);   

    // ROS Setup
    nh.initNode();
    nh.advertise(pub1);
    nh.advertise(pub2);
    nh.subscribe(sub);
} 

/**
 *****************************************************************************************************
 * main loop
 *****************************************************************************************************
 */
void loop() {
    // モータに入力を加える
    MotorL(w_l * 24); 
    MotorR(w_r*  24);

    // エンコーダのカウント
    update_left();
    update_right();

    // ROS Publish
    // 左右のencoder値の符号が異なるので、前進を+, 後進を-で統一した
    a.data = left_counter;
    b.data = right_counter;
    
    pub1.publish(&a);
    pub2.publish(&b);
  
    nh.spinOnce();
  //  delayMicroseconds(10);
    delay(10);
}

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

     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, LOW);

     digitalWrite(IN2_L, HIGH);

 }

 if (Pulse_Width1 < 0){

     Pulse_Width1=abs(Pulse_Width1);

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

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, LOW);

     digitalWrite(IN2_R, HIGH);

 }

 if (Pulse_Width2 < 0){

     Pulse_Width2=abs(Pulse_Width2);

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