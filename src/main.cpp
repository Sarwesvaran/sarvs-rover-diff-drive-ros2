#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h> 

#include <RMCS2303drive.h>
#include "CytronMotorDriver.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// --- CONFIGURATION ---
#define ODEM_RESET 1

// PINS
#define LEFT_PWM_PIN 32
#define LEFT_DIR_PIN 25
#define RIGHT_PWM_PIN 33
#define RIGHT_DIR_PIN 26
#define RMCS_RX_PIN 16
#define RMCS_TX_PIN 17
#define I2C_SDA 21
#define I2C_SCL 22

// ROBOT CONSTANTS
const float TICKS_PER_REV = 4680.0;
float WHEEL_RADIUS = 0.055;
float TRACK_WIDTH = 0.384;
float MAX_WHEEL_RAD_S = 20.94;

// TRIM
float LEFT_TRIM = 1.0;
float RIGHT_TRIM = 1.0;

// OBJECTS
CytronMD left_motor(PWM_DIR, LEFT_PWM_PIN, LEFT_DIR_PIN);
CytronMD right_motor(PWM_DIR, RIGHT_PWM_PIN, RIGHT_DIR_PIN);
RMCS2303 rmcs;
Adafruit_MPU6050 mpu; 

byte slave_id_left = 1;
byte slave_id_right = 2;

// ROS ENTITIES
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t pub_wheels;
rcl_publisher_t pub_imu; 
rcl_subscription_t subscriber;

geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Int32MultiArray msg_wheels;
sensor_msgs__msg__Imu msg_imu; 

int32_t memory[2];
#if ODEM_RESET
int32_t temp_encoder[2] = {0, 0};
bool reset_trigger = false;
#endif

bool mpu_ready = false; 

// INTERLEAVING VARIABLES
bool read_left_next = true; // Toggle switch
int32_t cached_left = 0;    // Store last known value
int32_t cached_right = 0;   // Store last known value

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void error_loop() {
  while (1) { digitalWrite(2, !digitalRead(2)); delay(500); }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  float wheel_left_rad_s = (linear_x - (angular_z * (TRACK_WIDTH / 2.0))) / WHEEL_RADIUS;
  float wheel_right_rad_s = (linear_x + (angular_z * (TRACK_WIDTH / 2.0))) / WHEEL_RADIUS;

  int left_pwm = (int)mapFloat(fabs(wheel_left_rad_s), 0.0, MAX_WHEEL_RAD_S, 0, 255);
  int right_pwm = (int)mapFloat(fabs(wheel_right_rad_s), 0.0, MAX_WHEEL_RAD_S, 0, 255);

  left_pwm *= LEFT_TRIM;
  right_pwm *= RIGHT_TRIM;

  if (left_pwm > 255) left_pwm = 255;
  if (right_pwm > 255) right_pwm = 255;

  if (wheel_left_rad_s > 0.01) left_motor.setSpeed(left_pwm);
  else if (wheel_left_rad_s < -0.01) left_motor.setSpeed(-left_pwm);
  else left_motor.setSpeed(0);

  if (wheel_right_rad_s > 0.01) right_motor.setSpeed(right_pwm);
  else if (wheel_right_rad_s < -0.01) right_motor.setSpeed(-right_pwm);
  else right_motor.setSpeed(0);

#if ODEM_RESET
  if (msg->angular.x == 1.0) reset_trigger = true;
#endif
}

void setup() {
  pinMode(2, OUTPUT);
  
  // 1. Init RMCS Serial
  Serial2.begin(9600, SERIAL_8N1, RMCS_RX_PIN, RMCS_TX_PIN);
  Serial2.setTimeout(2); // Very short timeout
  rmcs.begin(&Serial2, 9600);
  
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);

  // 2. Init I2C & MPU6050 
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    for(int i=0; i<5; i++){ digitalWrite(2, HIGH); delay(100); digitalWrite(2, LOW); delay(100); }
    mpu_ready = false; 
  } else {
    mpu_ready = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // 3. Init Micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) error_loop();
  if (rclc_node_init_default(&node, "esp32_driver", "", &support) != RCL_RET_OK) error_loop();

  rclc_publisher_init_default(&pub_wheels, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "value_wheels");

  rclc_publisher_init_default(&pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

  rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA);

  msg_wheels.data.capacity = 2;
  msg_wheels.data.size = 2;
  msg_wheels.data.data = memory;

  msg_imu.header.frame_id.data = (char * )malloc(20 * sizeof(char));
  sprintf(msg_imu.header.frame_id.data, "imu_link");
  msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);
  msg_imu.header.frame_id.capacity = 20;

  // Initial Read
  cached_left = rmcs.Position_Feedback(slave_id_left);
  cached_right = rmcs.Position_Feedback(slave_id_right);
#if ODEM_RESET
  temp_encoder[0] = cached_left;
  temp_encoder[1] = cached_right;
#endif
}

void loop() {
  
  // 1. INTERLEAVED READING (Only read ONE motor per loop)
  if (read_left_next) {
    long val = rmcs.Position_Feedback(slave_id_left);
    // Only update if value is valid (library returns -1 on timeout sometimes)
    // Assuming Position_Feedback returns valid long. 
    cached_left = val; 
    read_left_next = false; // Next loop, read right
  } else {
    long val = rmcs.Position_Feedback(slave_id_right);
    cached_right = val;
    read_left_next = true; // Next loop, read left
  }

  // 2. READ IMU
  if (mpu_ready) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    msg_imu.linear_acceleration.x = a.acceleration.x;
    msg_imu.linear_acceleration.y = a.acceleration.y;
    msg_imu.linear_acceleration.z = a.acceleration.z;

    msg_imu.angular_velocity.x = g.gyro.x;
    msg_imu.angular_velocity.y = g.gyro.y;
    msg_imu.angular_velocity.z = g.gyro.z;
    msg_imu.orientation.w = 1.0; 
  }

  // 3. PUBLISH
#if ODEM_RESET
  if (reset_trigger) {
    temp_encoder[0] = cached_left; temp_encoder[1] = cached_right;
    reset_trigger = false;
  }
  msg_wheels.data.data[0] = -1 * (cached_left - temp_encoder[0]);
  msg_wheels.data.data[1] = (cached_right - temp_encoder[1]);
#else
  msg_wheels.data.data[0] = -1 * cached_left;
  msg_wheels.data.data[1] = cached_right;
#endif

  rcl_publish(&pub_wheels, &msg_wheels, NULL);

  if (mpu_ready) {
    rcl_publish(&pub_imu, &msg_imu, NULL);
  }

  // 4. SPIN
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}