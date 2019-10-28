
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1
#define DEBUG 1
SSD1306AsciiWire oled;
#define LEFT_MOTOR_ENABLE 5
#define LEFT_MOTOR_A 6
#define LEFT_MOTOR_B 7
#define RIGHT_MOTOR_A 8
#define RIGHT_MOTOR_B 9
#define RIGHT_MOTOR_ENABLE 10
#define WIRE_SENSOR_LEFT A1
#define WIRE_SENSOR_RIGHT A0

#define BTN_FWD_1 4
#define BTN_FWD_2 11
#define BTN_FWD_3 12
#define BTN_BWD 13

#define FWD_SPEED_BASE_1X 100
#define FWD_SPEED_BASE_2X 180
#define FWD_SPEED_BASE_3X 250
#define BWD_SPEED_BASE -100

#define ERROR_GAIN 0.1
#define NUM_ADC_SAMPLES 20
#define MAX_SPEED 255
#define MIN_SPEED -255


struct Pose {
  byte fwd_btn_1;
  byte fwd_btn_2;
  byte fwd_btn_3;
  byte bwd_btn_1;
  int left_sensor;
  int right_sensor;
};

struct PayloadCommand {
  int leftSpeed;
  int rightSpeed;
};

struct GameData {
  long startTime;  
};

GameData gameData = {0};

void setup() {
  Wire.begin();
  Wire.setClock(400000L);  

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.displayRemap(true);
#endif // RST_PIN >= 0
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("BattlePoint v1.0"); 
  oled.println("Starting..."); 
  Serial.begin(38400);
  setupPins();
}

void startGame(){
  gameData.startTime = millis();
}

void setupPins(){
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  pinMode(WIRE_SENSOR_LEFT,INPUT);
  pinMode(WIRE_SENSOR_RIGHT,INPUT);
  pinMode(BTN_FWD_1,INPUT_PULLUP);
  pinMode(BTN_FWD_2,INPUT_PULLUP);
  pinMode(BTN_FWD_3,INPUT_PULLUP);
  pinMode(BTN_BWD,INPUT_PULLUP);
  
}


void loop() {
  Pose p = readPose();
  PayloadCommand pc = computePayloadCommand(p);
  handleCommand(pc);
  updateDisplay(p,pc);
  #if DEBUG
  updateSerial(p,pc);
  #endif
  updateDisplay(p,pc);
}

Pose readPose(){
  Pose p;
  p.fwd_btn_1 = readButton(BTN_FWD_1);
  p.fwd_btn_2 = readButton(BTN_FWD_2);
  p.fwd_btn_3 = readButton(BTN_FWD_3);
  p.bwd_btn_1 = readButton(BTN_BWD);
  p.left_sensor = readADCPinPeak(WIRE_SENSOR_LEFT,NUM_ADC_SAMPLES);
  p.right_sensor= readADCPinPeak(WIRE_SENSOR_RIGHT,NUM_ADC_SAMPLES);
  return p;
}

int readButton(int buttonPin){
  int i = digitalRead(buttonPin);
  return i == 0;
}

void updateDisplay(Pose p, PayloadCommand pc){
  int gameTimeSec = (millis() - gameData.startTime)/1000;
  char SPACE = ' ';
  oled.setCursor(0,1);
  oled.print("Time: ");
  oled.print(gameTimeSec);
  oled.print(" s");
  oled.clearToEOL();
  
  oled.setCursor(0,2);
  oled.print("BTNS:");
  oled.print(SPACE);
  oled.print(p.fwd_btn_1);
  oled.print(SPACE);
  oled.print(p.fwd_btn_2);
  oled.print(SPACE);
  oled.print(p.fwd_btn_3);
  oled.print(SPACE);
  oled.print(p.bwd_btn_1);
  oled.clearToEOL();
  oled.setCursor(0,3);
  oled.print("SENS: L:");
  oled.print(p.left_sensor);
  oled.print(" R:");   
  oled.print(p.right_sensor);
  oled.print(SPACE);
  oled.clearToEOL();
  oled.setCursor(0,4);
  oled.print("MTR: L ");  
  oled.print(pc.leftSpeed);
  oled.print(" R: ");
  oled.print(pc.rightSpeed);
  oled.clearToEOL();
}

void updateSerial(Pose p, PayloadCommand pc){
   char SPACE = ' ';
   Serial.print("B:");
   Serial.print(SPACE);
   Serial.print(p.fwd_btn_1);
   Serial.print(SPACE);
   Serial.print(p.fwd_btn_2);
   Serial.print(SPACE);
   Serial.print(p.fwd_btn_3);
   Serial.print(SPACE);
   Serial.print(p.bwd_btn_1);
   Serial.print(SPACE);
   Serial.print("L:");
   Serial.print(p.left_sensor);
   Serial.print(SPACE);
   Serial.print("R:");   
   Serial.print(p.right_sensor);
   Serial.print(SPACE);     
   Serial.print("LM:");
   Serial.print(pc.leftSpeed);
   Serial.print(SPACE);   
   Serial.print("RM:");
   Serial.print(pc.rightSpeed);
   Serial.println(SPACE);     
}

PayloadCommand computePayloadCommand(Pose p ){
  PayloadCommand pc;

  int nominal_speed = computeNominalSpeed(p);
  int sensor_delta = p.left_sensor - p.right_sensor;
  int correction = (float)sensor_delta * ERROR_GAIN;

  if ( nominal_speed > 0 ){

    pc.leftSpeed = min(nominal_speed - correction,MAX_SPEED);
    pc.rightSpeed = min(nominal_speed + correction,MAX_SPEED);
  }
  else if ( nominal_speed < 0 ){
    pc.leftSpeed = max(nominal_speed - correction,MIN_SPEED);
    pc.rightSpeed = max(nominal_speed + correction,MIN_SPEED);
  }
  else{
    pc.leftSpeed = 0;
    pc.rightSpeed = 0;
  }
  
  return pc;
}

void handleCommand(PayloadCommand mc ){
  setRightMotorSpeed(mc.rightSpeed );
  setLeftMotorSpeed(mc.leftSpeed);
}

int computeNominalSpeed(Pose p ){
  int num_fwd = num_fwd_pressed(p);
  if ( num_fwd == 3 ){
    return FWD_SPEED_BASE_3X;
  }  
  else if ( num_fwd == 2 ){
    return FWD_SPEED_BASE_2X;
  }
  else if ( num_fwd == 1 ){
    return FWD_SPEED_BASE_1X;
  }
  else if ( num_fwd == -1 ){
    return BWD_SPEED_BASE;
  }
  else{
    return 0;
  }
}

int num_fwd_pressed ( Pose p ){
  return p.fwd_btn_1 + p.fwd_btn_2 + p.fwd_btn_3 - p.bwd_btn_1;
}

void setRightMotorSpeed( int velocity){
  setMotorSpeed( RIGHT_MOTOR_A, RIGHT_MOTOR_B, RIGHT_MOTOR_ENABLE, velocity );
}

void setLeftMotorSpeed( int velocity){
  setMotorSpeed( LEFT_MOTOR_A, LEFT_MOTOR_B, LEFT_MOTOR_ENABLE, velocity );
}

void setMotorSpeed ( int pinA, int pinB, int pinEnable, int velocity ){
  int speed = abs(velocity);
  analogWrite(pinEnable, speed );
  if ( velocity > 0 ){
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);     
  }
  else{
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);    
  }
}

int readADCPinPeak(int pin, int samples ){
  int maxv = 0;
  int v = 0;
  for (int i=0;i<samples;i++){
    v = analogRead(pin);
    if ( v > maxv ) maxv = v;
  }
  return maxv;
}

