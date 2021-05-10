///////////////// Libs /////////////////

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h> //bt

///////////////// Define Pins /////////////////
// Used Pins: 2:13 , A5 , A6

// Motor M1 && M3, Lefts Sides (Front Wheels) && (Back wheels)
const uint8_t pwm_M1 = 3;   // ENA1 - Enable and PWM
const uint8_t fwd_M1 = 2;   // IN1 - Forward Drive (Front Wheels) && (Back wheels)
const uint8_t rwd_M1 = 4;   // IN2 - Reverse Drive (Front Wheels) && (Back wheels)
//-----------------
// Motor M2 && M4, Right Side (Front Wheels) && (Back wheels)
const uint8_t pwm_M2 = 5;   // ENB - Enable and PWM -> (Front Wheels) && (Back wheels) 
const uint8_t fwd_M2 = 7;   // IN3 - Forward Drive (Front Wheels) && (Back wheels)
const uint8_t rwd_M2 = 6;  // IN4 - Reverse Drive (Front Wheels) && (Back wheels)
// Bluetooth

SoftwareSerial bluetooth(A3, A2); //TX, RX (Bluetooth)
char buf;
String command = "";

///////////////// Define ajusts /////////////////

int speed = 250;  // Defines the base speed of the Rover
float P = 0.25;   // Proportion of rotation

////////////////  Functions Declare //////////////////
void allStop();
void allForward();
void allReverse();
void analyzeCommand(char c);

////////////////  SETUP  //////////////////

void setup() {
  // Set pins to motor driver (L298) to outputs
  pinMode(pwm_M1, OUTPUT);
  pinMode(fwd_M1, OUTPUT);
  pinMode(rwd_M1, OUTPUT);

  pinMode(pwm_M2, OUTPUT);
  pinMode(fwd_M2, OUTPUT);
  pinMode(rwd_M2, OUTPUT);
  // bluetooth module
  Serial.begin(9600);   
  // Default speed in HC-06 modules
  bluetooth.begin(9600);

}
////////////////  LOOP  //////////////////
void loop() {
  if (bluetooth.available()) {
    char c;
    c = bluetooth.read();
    analyzeCommand(c);
  }
}


//////////////// Start Functions //////////////////
void  analyzeCommand(char c)
{
  String s = "";
  s += c;

 
  if (s.startsWith("S")) { //9
//Representa as rodas acionadas (x)
    // xO
    // xO

    allStop();
  } else if (s.startsWith("F")) {// Front

    // Frente M1 && M3
    // Ox
    // Ox
    analogWrite(pwm_M1, speed / 1);
    digitalWrite(fwd_M1, HIGH);
    digitalWrite(rwd_M1, LOW);
    
    // Right M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 1);    
    digitalWrite(fwd_M2, HIGH);
    digitalWrite(rwd_M2, LOW);
         
  } else if (s.startsWith("G")) { // Front Left
    
    // Left M1 && M3
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 1);
    digitalWrite(fwd_M1, HIGH);
    digitalWrite(rwd_M1, LOW);

    // Right M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 0.5);
    digitalWrite(fwd_M2, HIGH);
    digitalWrite(rwd_M2, LOW);
    
  } else if (s.startsWith("I")) { //Front Right

    // Left M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 1);
    digitalWrite(fwd_M2, HIGH);
    digitalWrite(rwd_M2, LOW);

    // Right M1 && M3
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 0.5);
    digitalWrite(fwd_M1, HIGH);
    digitalWrite(rwd_M1, LOW);  
    
  } else if (s.startsWith("R")) { //Right

    // Left M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 1);
    digitalWrite(fwd_M2, LOW);
    digitalWrite(rwd_M2, HIGH);

    // Frente M1 && M3 
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 1);
    digitalWrite(fwd_M1, HIGH);
    digitalWrite(rwd_M1, LOW);

  } else if (s.startsWith("L")) { //Left
    // Left M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 1);
    digitalWrite(fwd_M2, HIGH);
    digitalWrite(rwd_M2, LOW);

    // Right M1 && M3
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 1);
    digitalWrite(fwd_M1, LOW);
    digitalWrite(rwd_M1, HIGH);
    

  }  else if (s.startsWith("B")) { //Back

    // Left M2 && M4 
    // xO
    // xO
    analogWrite(pwm_M2, speed / 1);
    digitalWrite(fwd_M2, LOW);
    digitalWrite(rwd_M2, HIGH);

    // Right M1 && M3
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 1);
    digitalWrite(fwd_M1, LOW);
    digitalWrite(rwd_M1, HIGH);

  } else if (s.startsWith("H")) { //Back Left

    // Left M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 0.5);
    digitalWrite(fwd_M2, LOW);
    digitalWrite(rwd_M2, HIGH);

    // Right M1 && M3
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 1);
    digitalWrite(fwd_M1, LOW);
    digitalWrite(rwd_M1, HIGH);
    
  } else if (s.startsWith("J")) { //3

    // Left M2 && M4
    // xO
    // xO
    analogWrite(pwm_M2, speed / 1);
    digitalWrite(fwd_M2, LOW);
    digitalWrite(rwd_M2, HIGH);

    // Right M1 && M3
    // 0x
    // 0x
    analogWrite(pwm_M1, speed / 0.5);
    digitalWrite(fwd_M1, LOW);
    digitalWrite(rwd_M1, HIGH);
  }

  else if (s.startsWith("0")) speed = 0;
  else if (s.startsWith("1")) speed = 50;
  else if (s.startsWith("2")) speed = 60;
  else if (s.startsWith("3")) speed = 70;
  else if (s.startsWith("4")) speed = 80;
  else if (s.startsWith("5")) speed = 100;
  else if (s.startsWith("6")) speed = 130;
  else if (s.startsWith("7")) speed = 150;
  else if (s.startsWith("8")) speed = 180;
  else if (s.startsWith("9")) speed = 220;
  else if (s.startsWith("q")) speed = 255;
  else if (s.startsWith("D")) speed = 0;

}

// All Stop

void allStop() {

  digitalWrite(fwd_M1, LOW);
  digitalWrite(rwd_M1, LOW);
  digitalWrite(fwd_M2, LOW);
  digitalWrite(rwd_M2, LOW);
  analogWrite(pwm_M1, 0);
  analogWrite(pwm_M2, 0);
}