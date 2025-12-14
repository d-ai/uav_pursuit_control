# UAV Pursuit Control using Vicon and Model Predictive Control

This repository contains a complete autonomous pursuit and control framework for a Parrot AR.Drone 2.0, developed during a research internship at the University of Alberta. The system uses Vicon motion capture for state estimation and implements both classical PID control and Linear Model Predictive Control (LMPC) for real-time UAV tracking.

The project demonstrates a full robotics pipeline: state estimation, reference generation, control, and execution on a real aerial robot.

---

## System Overview

The framework consists of three main components:

1. **State Estimation & Relative Pose**
   - Uses Vicon motion capture to estimate the world-frame and body-frame pose of two UAVs (pursuer and target)
   - Computes relative position and orientation in the target UAV’s body frame

2. **Reference Generation (Pursuit Logic)**
   - Generates a moving reference point offset from the target UAV
   - Publishes reference position, velocity, and yaw using a custom ROS message

3. **Control**
   - **PID Controller** 
     - Baseline controller for position and yaw tracking
     - Live gain tuning using OpenCV trackbars
   - **Linear Model Predictive Controller (LMPC)** 
     - 8-state linear model (position, yaw, velocities)
     - Finite-horizon optimization with input constraints
     - Includes a Luenberger observer for velocity estimation

Both controllers publish velocity and were tested on a real Parrot AR.Drone 2.0.

---

## Requirements

- ROS (tested with ROS Kinetic / Melodic)
- Parrot AR.Drone 2.0
- Vicon motion capture system
- Eigen
- OpenCV
- TF

---

## Notes on Reproducibility

Due to hardware and laboratory constraints, raw Vicon data and flight logs are not included. The code is provided for research and educational purposes and reflects a real-world UAV autonomy pipeline deployed on physical hardware.

---

## Author

**Debasmita Chatterjee**  
Research Internship – University of Alberta  


