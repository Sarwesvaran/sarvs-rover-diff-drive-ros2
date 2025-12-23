# sarvs-rover-diff-drive-ros2
ESP32 Differential Drive Robot (ROS 2 Humble)
This repository contains the firmware for a 2-wheel differential drive robot with a front castor wheel. It uses an ESP32 to bridge the gap between ROS 2 (micro-ROS) and the hardware.

The system uses Cytron MD drivers for motor power control (via PWM) and RMCS-2303 drivers for high-precision encoder feedback (via Hardware Serial).

🛠 Hardware Architecture
Controller: ESP32 (DevKit V1)

Motor Driver: Cytron MD series (connected via PWM/DIR)

Encoder Feedback: RMCS-2303 DC Servo Driver (connected via UART/Serial2)

Communication: micro-ROS over USB Serial

Chassis: 2 Motors + 1 Front Castor Wheel

Pin Mapping (ESP32)

Component	Function	ESP32 Pin
Left Motor	PWM	GPIO 32
Left Motor	Direction	GPIO 25
Right Motor	PWM	GPIO 33
Right Motor	Direction	GPIO 26
RMCS-2303	RX (Serial2)	GPIO 16
RMCS-2303	TX (Serial2)	GPIO 17
🚀 Software Architecture
The firmware implements differential drive kinematics to convert ROS 2 geometry_msgs/Twist messages into motor PWM signals.

ROS 2 Interface

Subscribed Topic: /cmd_vel (geometry_msgs/msg/Twist) - Controls robot movement.

Published Topic: /value_wheels (std_msgs/msg/Int32MultiArray) - Reports raw encoder ticks.

⚙️ Installation & Setup
1. Requirements

PlatformIO (VS Code)

micro-ROS Agent installed on your Raspberry Pi 5.

Libraries: * micro_ros_arduino

Cytron Motor Drivers Library

RMCS2303drive (included in /lib)

2. Building

In your PlatformIO project:

Bash
# Clean and build the project
pio run --target clean
pio run
3. Running the Agent (On Raspberry Pi 5)

Connect the ESP32 to your Pi via USB and run:

Bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
(Replace /dev/ttyUSB0 with your actual port)
 
📝 License
This project is open-source under the MIT License.

Pro-Tip for your Git:

You should also include a photo of your actual robot and a screenshot of the ros2 topic echo /value_wheels output to show that it is working!