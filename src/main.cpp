#define PINMOT_1 12 // D6 p
#define PINMOT_2 15 // D8 p
#define PINMOT_3 13 // D7 p
#define PINMOT_4 14 // D5 p


#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire, 0.25, 0.75);

float angle_pitch_output, angle_roll_output, angle_yaw_output;

unsigned long timer = 0;
unsigned long Time = 0;
float elapsedTime;

float Vbat;

float pitch_PID,roll_PID,yaw_PID;
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i;
const double twoX_kp = 0.9;//=5;      
const double twoX_ki = 0.003;//0.001;//=0.0025;
const double twoX_kd = 0;//=2;     
const double yaw_kp = 1.5; //=2;    
const double yaw_ki = 0; //=0.002;

float input_THROTTLE;
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle; 
#define WINDUP 50
#define FORCE_CONTROL 150

int motor_1, motor_2, motor_3, motor_4;

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"  //upload  [ctrl] + [shift] + p
#include <Arduino_JSON.h>
#include "wifiluca.h"


void setup() {
  pinMode(PINMOT_1,OUTPUT);
  pinMode(PINMOT_2,OUTPUT);
  pinMode(PINMOT_3,OUTPUT);
  pinMode(PINMOT_4,OUTPUT);
  digitalWrite(PINMOT_1, HIGH);
  digitalWrite(PINMOT_2, HIGH);
  digitalWrite(PINMOT_3, HIGH);
  digitalWrite(PINMOT_4, HIGH);

  analogWriteFreq(10000);

  Wire.begin();
  Wire.setClock(700000);

  Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {
      Serial.println("sensore trovato 1");
    }else{
      Wire.beginTransmission(0x69);
        if (Wire.endTransmission() == 0) {
          Serial.println("sensore trovato 2");
        }else{
          Serial.println("errore solito");
        }
    }
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);    // Seleziona il registro PWR_MGMT_1
  Wire.write(0x80);    // Scrivi 0x80 per eseguire il reset
  Wire.endTransmission();
  
  Serial.begin(250000);
  delay(100);

  mpu6050.begin();
  mpu6050.writeMPU6050(0x1A, 0x00); // filtro passa basso eliminato
  mpu6050.calcGyroOffsets(true);

  Serial.println();
  Serial.print("accX : ");Serial.print(mpu6050.getAccX());
  Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
  Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());

  Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
  Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
  Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());

  Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
  Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());

  Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
  Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
  Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
  
  Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());

  delay(100);

  input_THROTTLE = 0;
  roll_desired_angle = 0;
  pitch_desired_angle = 0;
  yaw_desired_angle = 0;

  initWiFi();
  initFS();
  initWebSocket();
  hostSite();
}

void loop() {
  Time = micros();
  ws.cleanupClients();  // Clean up WebSocket clients

  if(Time - timer > 1000){
    mpu6050.update();
    elapsedTime = (float)(Time - timer) / (float)1000000;
    timer = Time;

    angle_roll_output = mpu6050.getAngleX() - 7;
    angle_pitch_output = mpu6050.getAngleY() + 0.8;
    angle_yaw_output = mpu6050.getAngleZ();

    Vbat = 0.1 * (4.9 * analogRead(A0) / 1024.0) + 0.9 * Vbat;

    Serial.print("giro: "+String(angle_roll_output)+"\t"+String(angle_pitch_output)+"\t"+String(angle_yaw_output));

    roll_previous_error = roll_error;
    pitch_previous_error = pitch_error;

    roll_error  = angle_roll_output  - roll_desired_angle;
    pitch_error = angle_pitch_output - pitch_desired_angle;  
    yaw_error   = angle_yaw_output - yaw_desired_angle;  
      
    roll_pid_p = twoX_kp*roll_error;
    pitch_pid_p = twoX_kp*pitch_error;
    yaw_pid_p = yaw_kp*yaw_error;

    roll_pid_i  = constrain(roll_pid_i + (twoX_ki * roll_error * elapsedTime * (float)1000), -WINDUP, WINDUP);
    pitch_pid_i = constrain(pitch_pid_i + (twoX_ki * pitch_error * elapsedTime * (float)1000), -WINDUP, WINDUP);
    yaw_pid_i   = constrain(yaw_pid_i + (yaw_ki * yaw_error * elapsedTime * (float)1000), -WINDUP, WINDUP);

    roll_pid_d = twoX_kd*((roll_error - roll_previous_error)/elapsedTime);
    pitch_pid_d = twoX_kd*((pitch_error - pitch_previous_error)/elapsedTime);

    roll_PID  = constrain(roll_pid_p + roll_pid_i + roll_pid_d, -FORCE_CONTROL, FORCE_CONTROL);
    pitch_PID = constrain(pitch_pid_p + pitch_pid_i + pitch_pid_d, -FORCE_CONTROL, FORCE_CONTROL);
    yaw_PID   = constrain(yaw_pid_p + yaw_pid_i, -FORCE_CONTROL, FORCE_CONTROL);

    if(input_THROTTLE > 10){
      motor_1 = input_THROTTLE - roll_PID + pitch_PID - yaw_PID;
      motor_2 = input_THROTTLE + roll_PID + pitch_PID + yaw_PID;
      motor_3 = input_THROTTLE + roll_PID - pitch_PID - yaw_PID;
      motor_4 = input_THROTTLE - roll_PID - pitch_PID + yaw_PID;
    }else{
      motor_1 = 0;
      motor_2 = 0;
      motor_3 = 0;
      motor_4 = 0;

      yaw_desired_angle = angle_yaw_output;

      roll_pid_i = 0;
      pitch_pid_i = 0;
      yaw_pid_i = 0;
    }

    motor_1 = constrain(motor_1, 0, 255);
    motor_2 = constrain(motor_2, 0, 255);
    motor_3 = constrain(motor_3, 0, 255);
    motor_4 = constrain(motor_4, 0, 255);

    //motor_1=0;
    //motor_2=0;
    //motor_3=0;
    //motor_4=0;

    Serial.print("\tth: "+String(input_THROTTLE)+"\t"+String(roll_desired_angle)+"\t"+String(pitch_desired_angle));
    Serial.print("\tpid: "+String(roll_pid_i)+"\t"+String(pitch_pid_i));
    Serial.print("\tmot: "+String(motor_1)+"\t"+String(motor_2)+"\t"+String(motor_3)+"\t"+String(motor_4));
    Serial.print("\tVabt: "+String(Vbat));
    //Serial.print("\tt: "+String(elapsedTime*(float)1000000));
    Serial.println();

    analogWrite(PINMOT_1, 255-motor_1);
    analogWrite(PINMOT_2, 255-motor_2);
    analogWrite(PINMOT_3, 255-motor_3);
    analogWrite(PINMOT_4, 255-motor_4);
    
  }

  delayMicroseconds(1000);
}



/*  Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
 */
