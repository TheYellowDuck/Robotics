# Robotics — RoboCupJunior 2023

Code for an **autonomous maze-solving robot** built for the [RoboCupJunior](https://junior.robocup.org/) Maze category, competed at the **2023 World Championship in Bordeaux, France**.

---

## Overview

The robot navigates an unknown maze autonomously, detecting walls, colored rescue tiles, and black/white floor markings using an **OpenMV camera** and **LiDAR sensors**. It maps its path in real time and makes navigation decisions without any external input.

**Language:** MicroPython (OpenMV / pyb framework)

---

## Hardware

- **OpenMV Cam** — onboard vision processor running all navigation logic
- **LiDAR sensors** — wall detection and distance measurement on all four sides
- **Servo motors** — drive and steering control
- **Color sensor** — backup floor tile detection

---

## Key Modules

| File | Purpose |
|---|---|
| `Nav6.py` | Final navigation logic — wall detection, tile classification, maze traversal |
| `Control.py` | Motor control interface — forward, turn left/right/back, stop |
| `Sensor.py` | LiDAR sensor wrapper — reads distances and identifies wall positions |
| `CRC.py` | Communication error-checking using cyclic redundancy check |
| `motor2.py` | Low-level motor driver |
| `Stop.py` | Emergency stop routine |

Test files (`TestBlack.py`, `TestColor.py`, `TestSensing.py`) were used during hardware calibration.

---

## Navigation Approach

- **Color detection** in LAB color space — distinguishes red, green, yellow, blue, black, and white tiles using tuned thresholds
- **Wall mapping** — encodes each cell's four walls as a bitmask for path memory
- **Tile classification** — identifies rescue tiles, checkpoints, and floor type to trigger appropriate behavior
- **Iterative development** — Nav through Nav6 represent successive rewrites as the robot's behavior was refined through testing

---

## Competition

> **RoboCupJunior World Championship 2023 — Bordeaux, France**
> Maze category — fully autonomous navigation and rescue tile detection
