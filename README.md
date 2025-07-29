# Robotic Arm Control with Inverse Kinematics

## Overview

This project implements a **robotic arm controller** using an Arduino and multiple servo motors. The system is designed to perform **pick-and-place operations** by leveraging **inverse kinematics (IK)** for smooth and precise motion. The software is structured to prioritize **code modularity, motion smoothing, and mathematical correctness**.

The core functionality is triggered via a **joystick button**, initiating a predefined sequence where the arm moves to a target, grips an object, and places it at another location.

## Features

* **Inverse Kinematics (IK)**:
  Dynamically calculates joint angles for the shoulder, elbow, and wrist based on target (x, y) coordinates using trigonometric equations.

* **Smooth Servo Control**:
  Implements a `moveServoSmooth` function to avoid jerky movements by transitioning gradually between angles.

* **Configurable Pick-and-Place Sequence**:
  The `pickAndPlace()` function defines the automated motion sequence, which can be easily extended for more tasks.

* **Button-Triggered Execution**:
  The joystick button acts as a start trigger for the motion routine, with debounce handling.

* **Mathematical Safety**:
  Automatically constrains target points to the arm’s reachable workspace to prevent invalid motion.

## Software Structure

### Core Files

* **`main.ino`**
  Contains the setup, loop, and motion control logic. Key functions:

  * `setup()`: Initializes servos and sets them to default positions.
  * `loop()`: Waits for joystick input to trigger motion.
  * `moveServoSmooth()`: Smooth angle interpolation for servos.
  * `computeAndMoveIK()`: Calculates angles using IK and moves servos.
  * `pickAndPlace()`: Orchestrates the automated movement sequence.

### Libraries Used

* **`Servo.h`**: For controlling multiple servo motors.
* **`math.h`**: For trigonometric and angle computations (acos, atan2, degrees).

## How It Works

1. On startup, the servos initialize to neutral (90°) positions.
2. When the joystick button is pressed:

   * The arm rotates the base to the pickup location.
   * The IK function computes angles for reaching the object.
   * The gripper closes, lifts the object, rotates to the drop-off point, and releases the object.
3. The arm resets to a standby position.

## Configuration

* **Link lengths**:
  Defined by `L1` (31.0 units) and `L2` (27.0 units) to match your arm’s physical design.

* **Servo Pins**:

  ```
  Base: Pin 3  
  Shoulder: Pin 4  
  Elbow: Pin 5  
  Wrist: Pin 6  
  Gripper: Pin 7  
  Joystick Button: Pin 2 (INPUT_PULLUP)
  ```

* **Motion Speed**:
  Adjust `smoothDelay` (default 10ms) to speed up or slow down servo movement.

## Development Notes

* Written in **C++ for Arduino**, following a modular structure for easy debugging and extension.
* Emphasis on **mathematical correctness** using trigonometric IK rather than hardcoded angles.
* The sequence and positions can be easily adjusted by modifying the coordinates passed to `computeAndMoveIK()`.

## Future Enhancements

* Add **dynamic joystick-based manual control** for live movements.
* Integrate **trajectory planning** for smoother multi-point transitions.
* Implement **error detection** for out-of-reach targets.
* Add **EEPROM-based calibration** for servo zero positions.

## Getting Started

1. Install the Arduino IDE.
2. Connect the servos and joystick to the specified pins.
3. Upload the code to your Arduino board.
4. Press the joystick button to initiate the pick-and-place routine.
