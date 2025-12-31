# UAV Strategic Deconfliction System

## Overview
This project implements a strategic deconfliction system that verifies whether a UAV mission
can be safely executed in shared airspace by checking spatial and temporal conflicts
with other drones.

## Features
- Waypoint-based mission planning
- Spatial safety buffer enforcement
- Temporal overlap detection
- Conflict explanation with location & time
- 2D, 3D, and 4D (space + time) visualization
- Automated test suite

## How to Run
```bash
pip install -r requirements.txt
python uav_main_runner.py
