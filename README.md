# Ball-on-Plate System

## Overview

This project documents my work on a **Ball-on-Plate system**, made to help future researchers get up to speed quickly with **Dynamixel motors** and the control setup.  
The goal is to make it easier for anyone joining the project to get the system running fast.

The bigger research goal is to use **reinforcement learning** to control the plate.  
For now, I’m starting with **PID control** to prove the system is working properly and the hardware is all set up.

The setup includes:
- A plate that tilts using **Dynamixel servos**
- A **camera** for tracking the ball’s position
- A **controller** (like a Raspberry Pi or laptop) that handles vision and motor control

The loop works like this:
1. The camera finds where the ball is.  
2. The control algorithm figures out what plate angle, or torque is needed to move the ball.  
3. The Dynamixel motors tilt the plate.  
4. The process repeats quickly
---
## Hardware
### Dynamixel XM-430

The servo motors used in this project are very high quality, and I'll give some notes on them.
1. The motors can operate in three different control modes: Position - Velocity - Current(Torque)
2. Each motor may be connected completely in parallel because the half-duplex UART protocol lets you address each motor seperately
3. The motors are controlled through a board called the 'USB2DYNAMIXEL' which connects directly to the raspberry pi or computer. It generates the required half-duplex UART signals when used in conjunction with the Dynamixel SDK. This board only generates logic level signals, and another board supplies power to the motors.
