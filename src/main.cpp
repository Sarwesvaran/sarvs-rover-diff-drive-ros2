#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>

// Include your Modified Library (Hardware Serial Only)
#include <RMCS2303drive.h>
// Include Cytron Library
#include "CytronMotorDriver.h"
#include <math.h>

// --- CONFIGURATION ---
#define ODEM_RESET 1

// 1. PIN DEFINITIONS (ESP32 Best Practice)
// Cytron Motor Driver Pins
#define LEFT_PWM_PIN 32
#define LEFT_DIR_PIN 25
#define RIGHT_PWM_PIN 33
#define RIGHT_DIR_PIN 26

// RMCS-2303 Serial Pins (Hardware Serial 2)
#define RMCS_RX_PIN 16
#define RMCS_TX_PIN 17

// 2. ROBOT PHYSICAL CONSTANTS
// Max speed calculation: 1.5 m/s / 0.055 radius = ~27.27 rad/s
float MAX_WHEEL_RAD_S = 27.273;
float WHEEL_RADIUS = 0.055; // Meters
float TRACK_WIDTH = 0.384;  // Distance between left and right wheels (Meters)

// --- OBJECTS ---
// Cytron: PWM = PWM_PIN, DIR = DIR_PIN
CytronMD left_motor(PWM_DIR, LEFT_PWM_PIN, LEFT_DIR_PIN);
CytronMD right_motor(PWM_DIR, RIGHT_PWM_PIN, RIGHT_DIR_PIN);

// RMCS: Used for Encoders ONLY
RMCS2303 rmcs;
byte slave_id_left = 1;
byte slave_id_right = 2;

// --- ROS 2 ENTITIES ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Int32MultiArray msg_wheels;

// Variables
int32_t memory[2]; // Array to store encoder data
#if ODEM_RESET
int32_t temp_encoder[2] = {0, 0};
bool reset_trigger = false;
#endif

// --- HELPER FUNCTIONS ---
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void error_loop()
{
  while (1)
  {
    digitalWrite(2, !digitalRead(2)); // Flash LED on error
    delay(100);
  }
}

// --- CALLBACK: VELOCITY COMMAND ---
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // 1. DIFFERENTIAL DRIVE KINEMATICS
  // We only use Linear X (Forward/Back) and Angular Z (Turn)
  // Linear Y is ignored because a 2-wheel robot cannot move sideways.

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  // Calculate target angular velocity for each wheel (in radians/sec)
  // Left Wheel = (V - w * (Width/2)) / r
  float wheel_left_rad_s = (linear_x - (angular_z * (TRACK_WIDTH / 2.0))) / WHEEL_RADIUS;

  // Right Wheel = (V + w * (Width/2)) / r
  float wheel_right_rad_s = (linear_x + (angular_z * (TRACK_WIDTH / 2.0))) / WHEEL_RADIUS;

  // 2. MAP TO PWM (0-255)
  // We take the absolute value for mapping, then restore direction later
  int left_pwm = (int)mapFloat(fabs(wheel_left_rad_s), 0.0, MAX_WHEEL_RAD_S, 0, 255);
  int right_pwm = (int)mapFloat(fabs(wheel_right_rad_s), 0.0, MAX_WHEEL_RAD_S, 0, 255);

  // Cap PWM at 255
  if (left_pwm > 255)
    left_pwm = 255;
  if (right_pwm > 255)
    right_pwm = 255;

  // 3. APPLY TO MOTORS (With Direction)

  // Left Motor Logic
  if (wheel_left_rad_s > 0.01)
  {
    left_motor.setSpeed(left_pwm); // Forward
  }
  else if (wheel_left_rad_s < -0.01)
  {
    left_motor.setSpeed(-left_pwm); // Reverse (Cytron takes negative for reverse)
  }
  else
  {
    left_motor.setSpeed(0); // Stop
  }

  // Right Motor Logic
  if (wheel_right_rad_s > 0.01)
  {
    right_motor.setSpeed(right_pwm); // Forward
  }
  else if (wheel_right_rad_s < -0.01)
  {
    right_motor.setSpeed(-right_pwm); // Reverse
  }
  else
  {
    right_motor.setSpeed(0); // Stop
  }

#if ODEM_RESET
  // Special command to reset odometry
  if (msg->angular.x == 1.0)
  {
    reset_trigger = true;
  }
#endif
}

void setup()
{
  pinMode(2, OUTPUT); // Status LED
  // 1. Initialize Encoders (RMCS-2303)
  // Serial2 on ESP32 is usually pins 16(RX) and 17(TX)
  Serial2.begin(9600, SERIAL_8N1, RMCS_RX_PIN, RMCS_TX_PIN);
  rmcs.begin(&Serial2, 9600);

  // 2. Initialize Motors (Cytron)
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);

  // 3. Initialize Micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    error_loop();
  if (rclc_node_init_default(&node, "esp32_driver", "", &support) != RCL_RET_OK)
    error_loop();

  // Publisher: Wheel Encoders
  if (rclc_publisher_init_default(
          &publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
          "value_wheels") != RCL_RET_OK)
    error_loop();

  // Subscriber: CMD_VEL
  if (rclc_subscription_init_default(
          &subscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "cmd_vel") != RCL_RET_OK)
    error_loop();

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    error_loop();

  rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA);

  // Prepare Encoder Message Memory
  msg_wheels.data.capacity = 2;
  msg_wheels.data.size = 2;
  msg_wheels.data.data = memory;

#if ODEM_RESET
  // Get initial offsets
  temp_encoder[0] = rmcs.Position_Feedback(slave_id_left);
  temp_encoder[1] = rmcs.Position_Feedback(slave_id_right);
#endif
}

void loop()
{
  int32_t raw_left = 0;
  int32_t raw_right = 0;

  // READ ENCODERS (Hardware Serial - Fast & Reliable)
  raw_left = rmcs.Position_Feedback(slave_id_left);
  raw_right = rmcs.Position_Feedback(slave_id_right);

#if ODEM_RESET
  if (reset_trigger)
  {
    temp_encoder[0] = raw_left;
    temp_encoder[1] = raw_right;
    reset_trigger = false;
  }
  msg_wheels.data.data[0] = raw_left - temp_encoder[0];
  msg_wheels.data.data[1] = raw_right - temp_encoder[1];
#else
  msg_wheels.data.data[0] = raw_left;
  msg_wheels.data.data[1] = raw_right;
#endif

  // PUBLISH TO ROS 2
  rcl_ret_t ret = rcl_publish(&publisher, &msg_wheels, NULL);
  if (ret != RCL_RET_OK)
  {
    // Optional: Flash LED or handle error if publish fails
  }
  // SPIN ROS EXECUTOR
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  delay(10); // Small loop delay
}