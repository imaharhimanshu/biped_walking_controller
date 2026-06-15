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

## Build Instructions
This project uses the standard catkin build system. Do not build this repository directly; it must be compiled from the root of your workspace.

```bash
# Navigate to your workspace source directory
cd ~/catkin_ws/src

# Clone the repository
git clone [https://github.com/your-username/biped_walking_controller.git](https://github.com/your-username/biped_walking_controller.git)

# Navigate back to the workspace root and build
cd ~/catkin_ws
catkin_make
source devel/setup.bash
