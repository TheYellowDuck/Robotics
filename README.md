# Robotics — RoboCupJunior 2023

Code for an **autonomous maze-solving robot** built for the [RoboCupJunior](https://junior.robocup.org/) Maze category, competed at the **2023 World Championship in Bordeaux, France**. The robot navigates an unknown maze entirely on its own — perceiving walls, rescue tiles, and floor markings with an onboard camera and LiDAR, and deciding where to go with no external input.

## Overview

The robot navigates an unknown maze autonomously, detecting walls, colored rescue tiles, and black/white floor markings using an **OpenMV camera** and **LiDAR sensors**. It maps its path in real time and makes navigation decisions without any external input. All logic runs on-device in **MicroPython** (OpenMV / `pyb` framework).

## How It Works

Every cycle the robot runs a perception → decision → actuation loop entirely on the OpenMV camera. **Computer vision** in the LAB color space classifies the tile underneath it — distinguishing red, green, yellow, blue, black, and white using tuned thresholds — while **LiDAR** reads distances on all four sides to detect walls. Each maze cell's four walls are encoded as a **bitmask** so the robot can remember the layout it has explored, and tile classification (rescue tile, checkpoint, or floor type) triggers the appropriate behavior. Motor commands (forward, turn, reverse, stop) are issued through a control interface over a low-level driver, and inter-device messages are validated with a **cyclic redundancy check (CRC)**.

The navigation logic evolved through successive rewrites — `Nav` through `Nav6` — each refined against the physical robot during testing, with dedicated calibration scripts for color and sensing.

## Hardware

- **OpenMV Cam** — onboard vision processor running all navigation logic
- **LiDAR sensors** — wall detection and distance measurement on all four sides
- **Servo motors** — drive and steering control
- **Color sensor** — backup floor tile detection

## Key Modules

| File | Purpose |
|---|---|
| `Nav6.py` | Final navigation logic — wall detection, tile classification, maze traversal |
| `Control.py` | Motor control interface — forward, turn left/right/back, stop |
| `Sensor.py` | LiDAR sensor wrapper — reads distances and identifies wall positions |
| `CRC.py` | Communication error-checking using cyclic redundancy check |
| `motor2.py` | Low-level motor driver |
| `Stop.py` | Emergency stop routine |

The support modules (`Control.py`, `Sensor.py`, `motor2.py`, `Stop.py`, `CRC.py`) are documented helper scripts for motor control, sensing, and UART integrity. Test files (`TestBlack.py`, `TestColor.py`, `TestSensing.py`) were used during hardware calibration, and the earlier navigation drafts (`Nav` through `Nav5`) are kept to show the iterative development behind the final `Nav6.py`.

## Navigation Approach

- **Color detection** in LAB color space — distinguishes red, green, yellow, blue, black, and white tiles using tuned thresholds
- **Wall mapping** — encodes each cell's four walls as a bitmask for path memory
- **Tile classification** — identifies rescue tiles, checkpoints, and floor type to trigger appropriate behavior
- **Iterative development** — Nav through Nav6 represent successive rewrites as the robot's behavior was refined through testing

## Skills Demonstrated

- Autonomous navigation — real-time maze traversal with no external input
- Robotics & embedded programming — MicroPython on an OpenMV vision processor
- Computer vision — LAB color-space tile detection with tuned thresholds
- Image processing — onboard camera frame analysis for floor and rescue-tile classification
- Sensor integration & fusion — combining LiDAR, camera, and a color sensor for perception
- Motor control — drive and steering via servo motors over a low-level driver
- Maze mapping — per-cell four-wall bitmask encoding for path memory
- Algorithmic navigation — wall-aware decision logic for traversal
- Bitmasking & state encoding — compact representation of maze cells
- Serial communication — CRC (cyclic redundancy check) error detection
- Real-time control loop — perception, decision, and actuation on-device
- Iterative engineering — successive `Nav` rewrites refined through hardware testing
- Hardware calibration — dedicated color, black-line, and sensing test routines

## Tech Stack

- Python / MicroPython (OpenMV `pyb` framework)
- OpenMV Cam (onboard vision processor)
- LiDAR sensors (four-directional distance sensing)
- Servo motors + low-level motor driver
- Color sensor (backup tile detection)
- LAB color-space image processing
- CRC (cyclic redundancy check) for communication integrity

## Competition

> **RoboCupJunior World Championship 2023 — Bordeaux, France**
> Maze category — fully autonomous navigation and rescue tile detection
