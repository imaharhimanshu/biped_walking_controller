# Team Humanoid: Biped_Walking_Controller

## Overview
This repository contains the core ROS catkin workspace for Team Humanoid's bipedal walking robot. It includes the mathematical implementations for inverse kinematics, dynamic balance (ZMP), and hardware control. The project is built using a hybrid C++ and Python architecture to balance high-frequency control loops with rapid algorithmic prototyping.

## Repository Structure
*   **`config/`**: Configuration files and tunable parameters (YAML).
*   **`include/`**: C++ header files and dependencies.
*   **`launch/`**: ROS launch files for starting multiple nodes simultaneously.
*   **`scripts/`**: Python executable nodes.
*   **`src/`**: C++ source files for low-latency kinematics.
*   **`urdf/`**: Universal Robot Description Format files for physical modeling.

## Dependencies
*   Ubuntu 20.04 LTS
*   ROS Noetic 
*   Eigen (for kinematics matrix operations)


