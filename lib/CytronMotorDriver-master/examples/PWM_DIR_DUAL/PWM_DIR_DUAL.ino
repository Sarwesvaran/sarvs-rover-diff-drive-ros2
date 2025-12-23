/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************************
 * DESCRIPTION:
 *
 * This example shows how to drive 2 motors using the PWM and DIR pins with
 * 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Arduino D3  - Motor Driver PWM 1 Input
 * Arduino D4  - Motor Driver DIR 1 Input
 * Arduino D9  - Motor Driver PWM 2 Input
 * Arduino D10 - Motor Driver DIR 2 Input
 * Arduino GND - Motor Driver GND
 *
 *
 * AUTHOR   : Kong Wai Weng
 * COMPANY  : Cytron Technologies Sdn Bhd
 * WEBSITE  : www.cytron.io
 * EMAIL    : support@cytron.io
 *
 *******************************************************************************/

#include "CytronMotorDriver.h"

const int rightDir = 7;
const int leftDir = 8;

const int rightPWM = 10;
const int leftPWM = 9;

// Configure the motor driver.
CytronMD motor1(PWM_DIR, leftPWM, leftDir);    // PWM 1 = Pin 9, DIR 1 = Pin 8.
CytronMD motor2(PWM_DIR, rightPWM, rightDir);  // PWM 2 = Pin 10, DIR 2 = Pin 7.


// The setup routine runs once when you press reset.
void setup() {
}


// The loop routine runs over and over again forever.
void loop() {

  if (Serial.available() > 0) {
    String s = Serial.readString();

    if (s == "i") {
      motor1.setSpeed(128);  // Motor 1 runs forward at 50% speed.
      motor2.setSpeed(128);  // Motor 2 runs backward at 50% speed.
    }
    if (s == ",") {
      motor1.setSpeed(-128);  // Motor 1 runs forward at 50% speed.
      motor2.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
    }
    if (s == "j") {
      motor1.setSpeed(128);  // Motor 1 runs forward at 50% speed.
      motor2.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
    }
    if (s == "l") {
      motor1.setSpeed(-128);  // Motor 1 runs forward at 50% speed.
      motor2.setSpeed(128);  // Motor 2 runs backward at 50% speed.
    }
    if (s == "k") {
      motor1.setSpeed(0);  // Motor 1 runs forward at 50% speed.
      motor2.setSpeed(0);  // Motor 2 runs backward at 50% speed.
    }
  }
  delay(1000);
}