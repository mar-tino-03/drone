#define PINMOT_1 4
#define PINMOT_2 2
#define PINMOT_3 3
#define PINMOT_4 10
#define PINBAT   1

#include <Wire.h>
#include <MPU6050.h>
#define MPU6050_ADDR 0x69
MPU6050 mpu(MPU6050_ADDR);
#include <MadgwickAHRS.h>
Madgwick filter;

float acc_x, acc_y, acc_z;
float angle_roll_output, angle_pitch_output, angle_yaw_output;
float angle_roll_output_dot, angle_pitch_output_dot, angle_yaw_output_dot;
float lastYaw;

unsigned long Time = 0, Time_old = 0;
unsigned long timeBatteria = 0;
unsigned long TimeBAT = 0;
unsigned long ciclo_ISR = 0;
float elapsedTime;

float Vbat;

#define TAR_ROLL  - 5.40
#define TAR_PITCH - 1.80
#define TAR_YAW   - 180

float pitch_PID,roll_PID,yaw_PID;
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
float roll_dot_error, pitch_dot_error, yaw_dot_error;
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i, yaw_pid_d;
float twoX_kp = 5; //5
float twoX_ki = 0; //0.1
float twoX_kd = 0;
float yaw_kp = 10; //10;
float yaw_ki = 0;
float yaw_kd = 0;

#define ANELLO_VELOCITA true
float roll_dot_previous_error, pitch_dot_previous_error, yaw_dot_previous_error;
float roll_dot_pid_p, roll_dot_pid_d, roll_dot_pid_i, pitch_dot_pid_p, pitch_dot_pid_i, pitch_dot_pid_d, yaw_dot_pid_p, yaw_dot_pid_i, yaw_dot_pid_d;

float twoX_dot_kp = 3;
float twoX_dot_ki = 0;
float twoX_dot_kd = 0;
float yaw_dot_kp = 200;
float yaw_dot_ki = 0;
float yaw_dot_kd = 0;

bool modalita = false; // temporaneamente utilizzata per bloccare il seial print

float roll_desired_angle, pitch_desired_angle, yaw_desired_angle, yaw_dot_input_desired_angle, throttle_desired; 
float roll_dot_desired_angle, pitch_dot_desired_angle, yaw_dot_desired_angle;
#define FORCE_CONTROL 400
#define FORCE_CONTROL_TWOX 100
#define FORCE_CONTROL_YAW 100
#define FORCE_CONTROL_TWOX_DOT 400
#define FORCE_CONTROL_YAW_DOT 400

#define WINDUP 90

int motor_1, motor_2, motor_3, motor_4;

#define SIZE_DATA 1000          // save function
String dati[SIZE_DATA];
String dati_tot;
int dati_i=0;
bool writeInRam = false;
bool writeInFile = false;
unsigned long startTime;
boolean salta = true;

#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"  //upload  [ctrl] + [shift] + p    // pio run --target uploadfs
#include <Arduino_JSON.h>
#include "wifiluca.h"
#include "esp_timer.h"
#include "driver/gpio.h"
esp_timer_handle_t timer_control;

// Parametri PWM sfasato
const uint32_t freqHz = 1000;   // 1 kHz
const bool phase = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#include "pwmtino.h"

/*
#include <EEPROM.h>
#define EEPROM_SIZE 64
#define EEPROM_FLAG_ADDR 0
#define EEPROM_OFFSET_X 4
#define EEPROM_OFFSET_Y 8
#define EEPROM_OFFSET_Z 12
#define EEPROM_VALID_FLAG 123
#define PESOOFSET 0.1   //Fattore di aggiornamento dell'EMA*/


void IRAM_ATTR onTimer(void* arg) {
  Time = micros();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  angle_roll_output_dot =    gy / 131.0f; //avevo messo un filtro tempo fa
  angle_pitch_output_dot =   gx / 131.0f;
  angle_yaw_output_dot =   - gz / 131.0f;
  acc_x =   (ay / 8192.0f );
  acc_y =   (ax / 8192.0f );
  acc_z = - (az / 8192.0f );
  
  elapsedTime = (float)(Time - Time_old) * 1e-6f;
  Time_old = Time;

  filter.updateIMU(
    constrain(angle_roll_output_dot, -250, 250),
    constrain(angle_pitch_output_dot, -250, 250),
    constrain(angle_yaw_output_dot, -250, 250),
    acc_x,
    acc_y,
    acc_z
  );

  angle_roll_output = filter.getRoll() + TAR_ROLL;
  angle_pitch_output = filter.getPitch() + TAR_PITCH;

  float delta = filter.getYaw() + TAR_YAW - lastYaw;

  // correggi salto
  if (delta > 180.0f)   delta -= 360.0f;
  if (delta < -180.0f)  delta += 360.0f;

  angle_yaw_output += delta;
  lastYaw = filter.getYaw() + TAR_YAW;
  //if(abs(yaw_dot_input_desired_angle)>4) yaw_desired_angle += yaw_dot_input_desired_angle * 10 * elapsedTime;

  //___________________________________________________________________ pid angoli

  roll_error = roll_desired_angle - angle_roll_output;
  pitch_error = pitch_desired_angle - angle_pitch_output;
  yaw_error = yaw_desired_angle - angle_yaw_output;

  roll_pid_p = twoX_kp  * roll_error;
  pitch_pid_p = twoX_kp * pitch_error;
  yaw_pid_p = yaw_kp    * yaw_error;

  roll_pid_i  = constrain(roll_pid_i + (twoX_ki * roll_error * elapsedTime), -WINDUP, WINDUP);
  pitch_pid_i = constrain(pitch_pid_i + (twoX_ki * pitch_error * elapsedTime), -WINDUP, WINDUP);
  yaw_pid_i   = constrain(yaw_pid_i + (yaw_ki * yaw_error * elapsedTime), -WINDUP, WINDUP);

  roll_pid_d = -twoX_kd*((angle_roll_output_dot)/elapsedTime);
  pitch_pid_d = -twoX_kd*((angle_pitch_output_dot)/elapsedTime);
  yaw_pid_d = yaw_kd*((angle_yaw_output_dot)/elapsedTime);

  
  //___________________________________________________________________ pid velocità angolari
  
  roll_dot_desired_angle = constrain(roll_pid_p + roll_pid_i + roll_pid_d, -FORCE_CONTROL_TWOX, FORCE_CONTROL_TWOX);
  pitch_dot_desired_angle = constrain(pitch_pid_p + pitch_pid_i + pitch_pid_d, -FORCE_CONTROL_TWOX, FORCE_CONTROL_TWOX);
  yaw_dot_desired_angle = abs(yaw_dot_input_desired_angle)>8 ? yaw_dot_input_desired_angle : 0;
  //yaw_dot_desired_angle = constrain(yaw_pid_p + yaw_pid_i + yaw_pid_d, -FORCE_CONTROL_YAW, FORCE_CONTROL_YAW);
  //yaw_dot_desired_angle = abs(yaw_dot_input_desired_angle)>4 ? yaw_dot_input_desired_angle : 0;

  roll_dot_previous_error = roll_dot_error;
  pitch_dot_previous_error = pitch_dot_error;
  yaw_dot_previous_error =  yaw_dot_error;

  roll_dot_error = roll_dot_desired_angle - angle_roll_output_dot;
  pitch_dot_error = pitch_dot_desired_angle - angle_pitch_output_dot;
  yaw_dot_error =  - (yaw_dot_desired_angle - angle_yaw_output_dot);

  roll_dot_pid_p =  twoX_dot_kp * roll_dot_error;
  pitch_dot_pid_p = twoX_dot_kp * pitch_dot_error;
  yaw_dot_pid_p =   yaw_dot_kp  * yaw_dot_error;

  roll_dot_pid_i  = constrain(roll_dot_pid_i + (twoX_dot_ki * roll_dot_error * elapsedTime), -WINDUP, WINDUP);
  pitch_dot_pid_i = constrain(pitch_dot_pid_i + (twoX_dot_ki * pitch_dot_error * elapsedTime), -WINDUP, WINDUP);
  yaw_dot_pid_i   = constrain(yaw_dot_pid_i + (yaw_dot_ki * yaw_dot_error * elapsedTime), -WINDUP, WINDUP);

  roll_dot_pid_d = twoX_dot_kd*((roll_dot_error - roll_dot_previous_error)/elapsedTime);
  pitch_dot_pid_d = twoX_dot_kd*((pitch_dot_error - pitch_dot_previous_error)/elapsedTime);
  yaw_dot_pid_d = yaw_dot_kd*((yaw_dot_error - yaw_dot_previous_error)/elapsedTime);

  
  //___________________________________________________________________ somma contributi

  if(ANELLO_VELOCITA){
    roll_PID  = constrain(roll_dot_pid_p + roll_dot_pid_i + roll_dot_pid_d, -FORCE_CONTROL_TWOX_DOT, FORCE_CONTROL_TWOX_DOT);
    pitch_PID = constrain(pitch_dot_pid_p + pitch_dot_pid_i + pitch_dot_pid_d, -FORCE_CONTROL_TWOX_DOT, FORCE_CONTROL_TWOX_DOT);
    yaw_PID   = constrain(yaw_dot_pid_p + yaw_dot_pid_i + yaw_dot_pid_d, -FORCE_CONTROL_YAW_DOT, FORCE_CONTROL_YAW_DOT);
  }else{
    roll_PID  = constrain(roll_pid_p + roll_pid_i + roll_pid_d, -FORCE_CONTROL, FORCE_CONTROL);
    pitch_PID = constrain(pitch_pid_p + pitch_pid_i + pitch_pid_d, -FORCE_CONTROL, FORCE_CONTROL);
    yaw_PID   = constrain(yaw_pid_p + yaw_pid_i + yaw_pid_d, -FORCE_CONTROL, FORCE_CONTROL);
  }

  if(throttle_desired > 110 && abs(angle_roll_output)<110 && abs(angle_pitch_output)<110){
    motor_1 = throttle_desired - roll_PID  - pitch_PID - yaw_PID;
    motor_2 = throttle_desired + roll_PID  + pitch_PID - yaw_PID;
    motor_3 = throttle_desired + roll_PID  - pitch_PID + yaw_PID;
    motor_4 = throttle_desired - roll_PID  + pitch_PID + yaw_PID;
  }else if(throttle_desired > 10 && abs(angle_roll_output)<110 && abs(angle_pitch_output)<110){
    motor_1 = throttle_desired + (- roll_PID  - pitch_PID - yaw_PID) * (throttle_desired-10) / 100;
    motor_2 = throttle_desired + (+ roll_PID  + pitch_PID - yaw_PID) * (throttle_desired-10) / 100;
    motor_3 = throttle_desired + (+ roll_PID  - pitch_PID + yaw_PID) * (throttle_desired-10) / 100;
    motor_4 = throttle_desired + (- roll_PID  + pitch_PID + yaw_PID) * (throttle_desired-10) / 100;
  }else{
    motor_1 = 0;
    motor_2 = 0;
    motor_3 = 0;
    motor_4 = 0;

    //yaw_desired_angle = angle_yaw_output;
    roll_pid_i = 0;  // roll_pid_i *= 0.95;
    pitch_pid_i = 0;
    yaw_pid_i = 0;
    roll_dot_pid_i = 0;
    pitch_dot_pid_i = 0;
    yaw_dot_pid_i = 0;
  }

  motor_1 = constrain(motor_1, 0, 1023);
  motor_2 = constrain(motor_2, 0, 1023);
  motor_3 = constrain(motor_3, 0, 1023);
  motor_4 = constrain(motor_4, 0, 1023);

  set_power(motor_1, motor_2, motor_3, motor_4);


  if(true){
    Serial.print("t: "+String(ciclo_ISR));
    Serial.print("\tmot_pid: "+String(roll_PID)+"\teri: "+String(roll_pid_i));
    //Serial.print("\tax: "+String(acc_x)+"\tay: "+String(acc_y)+"\taz: "+String(acc_z));
    Serial.print("\tth: "+String(throttle_desired)+"\tr:"+String(angle_roll_output)+"\tp:"+String(angle_pitch_output)+"\ty:"+String(angle_yaw_output));
    //Serial.print("\trd: "+String(angle_roll_output_dot)+"\tpd: "+String(angle_pitch_output_dot)+"\tyd: "+String(angle_yaw_output_dot));
    Serial.print("\tmot: "+String(motor_1)+"\t"+String(motor_2)+"\t"+String(motor_3)+"\t"+String(motor_4));
    //Serial.print("\tyd: "+String(angle_yaw_output_dot)+"\tpid_d: "+String(yaw_dot_pid_p)+"\tdes_d: "+String(yaw_dot_desired_angle));
    //Serial.print("\ter: "+String(roll_error)+"\tep: "+String(pitch_error)+"\tey: "+String(yaw_error));
    //Serial.print("\tVabt: "+String(Vbat));
    //if(roll_dot_pid_p > FORCE_CONTROL || -FORCE_CONTROL > roll_dot_pid_p) Serial.print("\n\nerrore sat\n\n");
    //Serial.print("\tt: "+String(micros() - Time)); //840
    Serial.println();
  }

  //ledcWrite(0, 1023-motor_1);
  //ledcWrite(1, 1023-motor_2);
  //ledcWrite(2, 1023-motor_3);
  //ledcWrite(3, 1023-motor_4);
  

  if(writeInRam && salta){ //write in file.txt
    dati[dati_i] = "t: "+String(Time/1000 - startTime)+
                  //"\tmot: "+String(motor_1)+"\t"+String(motor_2)+"\t"+String(motor_3)+"\t"+String(motor_4)+
                  "\tr: "+String(angle_roll_output)+"\trd: "+String(angle_roll_output_dot)+"\tu_r: "+String(roll_PID)+
                  //"\tyd: "+String(angle_yaw_output_dot)+
                  //"\ty: "+String(angle_yaw_output)+//"\tdes_dot: "+String(yaw_dot_desired_angle)+"\tdot_pid_p: "+String(yaw_dot_pid_p)+
                  "\n";
    if(dati_i < SIZE_DATA) dati_i++; 
  }
  salta = !salta;

  //Serial.print("\tt: "+String(micros() - Time));
  ciclo_ISR = micros() - Time;
}

void setup() {

  init_pwm(PINMOT_1, PINMOT_2, PINMOT_3, PINMOT_4, freqHz, phase);

  pinMode(PINBAT, INPUT);

  Serial.begin(115200);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();       // equivalente a begin()
  delay(10);
  mpu.setDLPFMode(3); // DLPF = 42 Hz mpu.setDLPFMode(0); disattivato
  delay(10);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // +-4g
  mpu.CalibrateGyro(100);

  filter.begin(500);
  
  throttle_desired = 0;
  roll_desired_angle = 0;
  pitch_desired_angle = 0;
  yaw_desired_angle = 0;
  yaw_dot_input_desired_angle = 0;
  roll_dot_desired_angle = 0;
  pitch_dot_desired_angle = 0;
  yaw_dot_desired_angle = 0;


  initWiFi();
  initFS();
  initWebSocket();
  hostSite();


  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &onTimer,        // Funzione da chiamare
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "MyTimer_control"
  };

  esp_timer_create(&periodic_timer_args, &timer_control);
  esp_timer_start_periodic(timer_control, 2000);
}

void loop() {
  TimeBAT = micros();
  ws.cleanupClients();  // Clean up WebSocket clients
  Vbat = 0.9*Vbat+0.1*analogRead(PINBAT) / 4096 * 6;
  if(TimeBAT - timeBatteria > 1000000){
    timeBatteria = TimeBAT;
    JSONVar invio;
    invio["batteria"] = Vbat;
    inviaDatiUtenti(invio);

    /*Serial.print("\tt: "+String(ciclo_ISR));
    Serial.print("\tth: "+String(throttle_desired)+"\tr:"+String(angle_roll_output)+"\tp:"+String(angle_pitch_output)+"\ty:"+String(angle_yaw_output));
    //Serial.print("\trd: "+String(angle_roll_output_dot)+"\tpd: "+String(angle_pitch_output_dot)+"\tyd: "+String(angle_yaw_output_dot));
    Serial.print("\tmot: "+String(motor_1)+"\t"+String(motor_2)+"\t"+String(motor_3)+"\t"+String(motor_4));
    //Serial.print("\tyd: "+String(angle_yaw_output_dot)+"\tpid_d: "+String(yaw_dot_pid_p)+"\tdes_d: "+String(yaw_dot_desired_angle));
    
    Serial.print("\tVabt: "+String(Vbat));
    //if(roll_dot_pid_p > FORCE_CONTROL || -FORCE_CONTROL > roll_dot_pid_p) Serial.print("\n\nerrore sat\n\n");
    Serial.println();*/
  }

  if(writeInFile == true){
    for(int i=0; i<dati_i; i++){
      dati_tot += dati[i];
    }
    writeFile(LittleFS, "/debug.txt", dati_tot.c_str());
    dati_i = 0;
    dati_tot = "";
    writeInFile = false;
  }
}
/*
void caricaOffset(bool get){//carica in variabili
  EEPROM.begin(EEPROM_SIZE); // Inizializza EEPROM con una dimensione adeguata
  
  byte flag;
  EEPROM.get(EEPROM_FLAG_ADDR, flag);

  if (get && flag == EEPROM_VALID_FLAG) {
    float offsetX;
    float offsetY;
    float offsetZ;
    EEPROM.get(EEPROM_OFFSET_X, offsetX);
    EEPROM.get(EEPROM_OFFSET_Y, offsetY);
    EEPROM.get(EEPROM_OFFSET_Z, offsetZ);

    if(abs(offsetX - mpu6050.getGyroXoffset()) < 0.5 && 
       abs(offsetY - mpu6050.getGyroYoffset()) < 0.5 && 
       abs(offsetZ - mpu6050.getGyroZoffset()) < 0.1 ){

      offsetX = (float)PESOOFSET * mpu6050.getGyroXoffset() + (float)(1 - PESOOFSET) * offsetX;
      offsetY = (float)PESOOFSET * mpu6050.getGyroYoffset() + (float)(1 - PESOOFSET) * offsetY;
      offsetZ = (float)PESOOFSET * mpu6050.getGyroZoffset() + (float)(1 - PESOOFSET) * offsetZ;

    }    

    // forzo la mia calibrazione
    //offsetX = 2.8220186;
    //offsetY = 4.0885192;
    //offsetZ = -0.1799488;
  
    uploadOffset(offsetX, offsetY, offsetZ);

    Serial.print("\n\nX: ");
    Serial.print(offsetX, 6);
    Serial.print("\nY: ");
    Serial.print(offsetY, 6);
    Serial.print("\nZ: ");
    Serial.print(offsetZ, 6);
    Serial.print("\n\n");
  } else {
    uploadOffset(mpu6050.getGyroXoffset(), mpu6050.getGyroYoffset(), mpu6050.getGyroZoffset());
  }
  EEPROM.end();
}
/*
void uploadOffset(float newX, float newY, float newZ){
    EEPROM.put(EEPROM_OFFSET_X, newX);
    EEPROM.put(EEPROM_OFFSET_Y, newY);
    EEPROM.put(EEPROM_OFFSET_Z, newZ);
    EEPROM.put(EEPROM_FLAG_ADDR, EEPROM_VALID_FLAG);  // flag valido
    Serial.println();
    Serial.println("X : " + String(newX));
    Serial.println("Y : " + String(newY));
    Serial.println("Z : " + String(newZ));
    mpu6050.setGyroOffsets(newX, newY, newZ);

    EEPROM.commit();
}*/