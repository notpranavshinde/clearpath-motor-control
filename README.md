# ClearPath Motor Control

This project provides an Arduino sketch to control a ClearPath SDSK motor for linear motion with velocity ramping. It allows you to specify a target speed, distance, and acceleration to move a load linearly.

## Features

*   Controls a ClearPath motor using Step/Direction signals.
*   Implements trapezoidal velocity profiling (ramp-up, cruise, ramp-down).
*   Configurable speed, distance, and acceleration in metric units.
*   Calculates and displays move parameters in both metric and imperial units.
*   Serial interface to trigger the motor movement.

## How to Use

1.  **Hardware Setup:**
    *   Connect the ClearPath motor to your Arduino as follows:
        *   `STEP+` (Black)  -> Arduino Pin 2
        *   `DIR+`  (White)  -> Arduino Pin 3
        *   `EN+`   (Blue)   -> Arduino Pin 4 (in series with a toggle switch)
        *   `STEP-`/`DIR-`/`EN-` -> Arduino GND
    *   The toggle switch on the `EN+` line is used to enable/disable the motor.

2.  **Software Setup:**
    *   Open the `.ino` sketch file in the Arduino IDE.
    *   Install the `AccelStepper` library. You can find it in the Arduino Library Manager.
    *   Modify the following constants in the code to match your desired movement profile:
        *   `SPEED_M_S`: The desired linear speed in meters per second.
        *   `DIST_M`: The desired linear distance in meters. A negative value will reverse the direction.
        *   `ACCEL_M_S2`: The desired linear acceleration in meters per second squared.

3.  **Running the Code:**
    *   Upload the sketch to your Arduino. **Keep the motor enable switch OPEN during the upload.**
    *   Open the Serial Monitor at 115200 baud.
    *   Close the enable switch to power the motor.
    *   Press Enter in the Serial Monitor to trigger the movement.

## Dependencies

*   [AccelStepper Library](https://www.airspayce.com/mikem/arduino/AccelStepper/)

