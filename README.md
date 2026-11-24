Self-Balancing Robot (ESP32 + MPU6050)


This project is a two-wheeled inverted pendulum robot that keeps itself upright by continuously correcting its tilt. 
Like a broomstick balanced on a palm, the robot is naturally unstable. It stays up only because the wheels move under it fast enough to cancel any lean.

How it works

An MPU6050 inertial sensor measures the robot's tilt using two sources.
The accelerometer gives an absolute tilt angle but is noisy and gets disturbed when the robot accelerates. 
The gyroscope gives a smooth rotation rate but drifts over time. A Kalman filter fuses the two into one clean, drift-corrected angle 
estimate, and it also tracks the gyro's bias as it changes. The ESP32-WROOM reads the sensor 200 times per second, runs the filter, and feeds the result into a PID controller.
The controller outputs a motor command proportional to the tilt, plus an integral term to remove slow drift and a derivative term from the filtered gyro rate to damp oscillation. 
A motor driver turns that command into PWM signals for the two DC gear motors. If the robot tips past a safe angle, the motors cut off until it is held upright again.

Main components

ESP32-WROOM microcontroller (control loop and processing)
MPU6050 6-axis IMU (accelerometer and gyroscope over I2C)
Dual H-bridge motor driver (TB6612 )
Two DC gear motors with wheels
Battery pack and chassis
<img width="720" height="696" alt="WhatsApp Image 2026-10-28 at 6 44 36 PM(1)" src="https://github.com/user-attachments/assets/92e15f36-1186-4039-8271-740b598ca20d" />


https://github.com/user-attachments/assets/e76c078c-ecb7-4e5d-806f-904cfb303a54

<img width="661" height="856" alt="WhatsApp Image 2026-10-28 at 6 44 36 PM" src="https://github.com/user-attachments/assets/2322155e-6a70-4e96-923a-0e9c5e44e527" />

Key features

Noise reduction in layers: the sensor's hardware low-pass filter, gyro offset calibration at startup, the Kalman filter, and rejection of the accelerometer during hard acceleration
Fixed-rate 200 Hz control loop for consistent timing
PID control with an anti-windup limit and compensation for the motors' dead zone
Fall detection and a safe restart
Live tuning of Kp, Ki, Kd and the balance setpoint over the serial monitor, without reflashing

Concepts it covers

Sensor fusion and state estimation, feedback control of an unstable system, PWM motor control, I2C communication, and real-time embedded firmware.
