#!/usr/bin/env python3
"""
Walking Pattern Generator for Humanoid Robot
Extends the balance controller with walking capabilities
"""

import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class WalkingParameters:
    step_length: float = 0.265  # meters
    step_duration: float = 1.1  # seconds
    double_support_ratio: float = 0.2  # 20% of step duration
    com_height: float = 0.87  # meters (zc from paper)
    foot_width: float = 0.1  # meters
    foot_length: float = 0.2  # meters
    control_dt: float = 0.005  # 5ms control cycle

class WalkingPatternGenerator:
    def __init__(self, params: WalkingParameters):
        self.params = params
        self.gravity = 9.81
        self.omega = math.sqrt(self.gravity / params.com_height)  # Natural frequency
        
    def generate_zmp_trajectory(self, num_steps: int) -> Tuple[np.ndarray, np.ndarray]:
        """Generate ZMP trajectory using cubic polynomials"""
        total_time = num_steps * self.params.step_duration
        time_points = np.arange(0, total_time, self.params.control_dt)
        
        zmp_x = np.zeros(len(time_points))
        zmp_y = np.zeros(len(time_points))
        
        for step in range(num_steps):
            step_start = step * self.params.step_duration
            step_end = (step + 1) * self.params.step_duration
            ds_duration = self.params.double_support_ratio * self.params.step_duration
            
            # Single support phase ZMP (center of support foot)
            if step % 2 == 0:  # Right foot support
                foot_center_y = -0.1  # Right foot position
            else:  # Left foot support
                foot_center_y = 0.1   # Left foot position
                
            # Find time indices for this step
            step_mask = (time_points >= step_start) & (time_points < step_end)
            step_indices = np.where(step_mask)[0]
            
            if len(step_indices) > 0:
                # ZMP trajectory within foot polygon during single support
                zmp_y[step_indices] = foot_center_y
                
                # Forward ZMP progression during step
                step_progress = (time_points[step_indices] - step_start) / self.params.step_duration
                zmp_x[step_indices] = step * self.params.step_length + step_progress * 0.1
        
        return time_points, np.column_stack([zmp_x, zmp_y])
    
    def generate_com_trajectory(self, zmp_trajectory: np.ndarray, time_points: np.ndarray) -> np.ndarray:
        """Generate CoM trajectory from ZMP using Linear Inverted Pendulum"""
        com_trajectory = np.zeros_like(zmp_trajectory)
        
        for axis in range(2):  # x and y axes
            # Initial conditions (robot starts at rest)
            com_pos = zmp_trajectory[0, axis]
            com_vel = 0.0
            
            com_trajectory[0, axis] = com_pos
            
            # Integrate LIPM dynamics: x_ddot = omega^2 * (x - p)
            for i in range(1, len(time_points)):
                dt = self.params.control_dt
                acceleration = self.omega**2 * (com_pos - zmp_trajectory[i-1, axis])
                com_vel += acceleration * dt
                com_pos += com_vel * dt
                com_trajectory[i, axis] = com_pos
                
        return com_trajectory
    
    def generate_foot_trajectories(self, num_steps: int, time_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Generate foot position/orientation trajectories"""
        num_points = len(time_points)
        
        # Initialize foot trajectories [x, y, z, roll, pitch, yaw]
        right_foot = np.zeros((num_points, 6))
        left_foot = np.zeros((num_points, 6))
        
        # Initial foot positions
        right_foot[:, 1] = -0.1  # Right foot y-position
        left_foot[:, 1] = 0.1    # Left foot y-position
        
        step_height = 0.05  # 5cm swing height
        
        for step in range(num_steps):
            step_start_time = step * self.params.step_duration
            step_end_time = (step + 1) * self.params.step_duration
            
            # Find time indices for this step
            step_mask = (time_points >= step_start_time) & (time_points < step_end_time)
            step_indices = np.where(step_mask)[0]
            
            if len(step_indices) == 0:
                continue
                
            step_time_local = time_points[step_indices] - step_start_time
            step_progress = step_time_local / self.params.step_duration
            
            if step % 2 == 0:  # Right foot swing
                # Left foot remains on ground
                target_x = (step + 1) * self.params.step_length
                
                # Swing foot trajectory (sinusoidal)
                right_foot[step_indices, 0] = target_x * step_progress
                right_foot[step_indices, 2] = step_height * np.sin(np.pi * step_progress)
                
            else:  # Left foot swing
                # Right foot remains on ground  
                target_x = (step + 1) * self.params.step_length
                
                # Swing foot trajectory
                left_foot[step_indices, 0] = target_x * step_progress
                left_foot[step_indices, 2] = step_height * np.sin(np.pi * step_progress)
        
        return right_foot, left_foot


class ZMPDelayModel:
    """Models ZMP delay as first-order system: p = 1/(1+sTp) * pd"""
    
    def __init__(self, time_constant: float = 0.05):
        self.Tp = time_constant
        self.previous_zmp = np.array([0.0, 0.0])
        
    def update(self, desired_zmp: np.ndarray, dt: float) -> np.ndarray:
        """Update actual ZMP based on desired ZMP with delay"""
        # First-order filter: p_dot = (pd - p) / Tp
        zmp_rate = (desired_zmp - self.previous_zmp) / self.Tp
        actual_zmp = self.previous_zmp + zmp_rate * dt
        self.previous_zmp = actual_zmp.copy()
        return actual_zmp


class GroundFrameManager:
    """Manages dynamic ground reference frame as described in the paper"""
    
    def __init__(self):
        self.current_frame_origin = np.array([0.0, 0.0, 0.0])
        self.current_frame_orientation = 0.0  # yaw angle
        
    def update_frame(self, right_foot_pos: np.ndarray, left_foot_pos: np.ndarray, 
                    right_foot_contact: bool, left_foot_contact: bool):
        """Update ground frame based on support phase"""
        
        if right_foot_contact and not left_foot_contact:
            # Single support on right foot
            self.current_frame_origin = right_foot_pos.copy()
            self.current_frame_orientation = 0.0  # Assume aligned with right foot
            
        elif left_foot_contact and not right_foot_contact:
            # Single support on left foot  
            self.current_frame_origin = left_foot_pos.copy()
            self.current_frame_orientation = 0.0  # Assume aligned with left foot
            
        elif right_foot_contact and left_foot_contact:
            # Double support - midpoint between feet
            self.current_frame_origin = (right_foot_pos + left_foot_pos) / 2.0
            self.current_frame_orientation = 0.0  # Average orientation
            
    def transform_to_ground_frame(self, world_point: np.ndarray) -> np.ndarray:
        """Transform point from world frame to current ground frame"""
        return world_point - self.current_frame_origin


class CompleteWalkingSystem:
    """Integrates pattern generation with LQR control"""
    
    def __init__(self, walking_params: WalkingParameters):
        self.params = walking_params
        self.pattern_generator = WalkingPatternGenerator(walking_params)
        self.zmp_delay_model = ZMPDelayModel()
        self.ground_frame = GroundFrameManager()
        
        # LQR controller gains (from your implementation)
        # Poles at (-13, -3, -ωc) as specified in paper
        omega_c = math.sqrt(9.81 / walking_params.com_height)
        self.lqr_gains = np.array([-2.67, -0.34, -20.0])  # [kp, kd, kzmp]
        
    def generate_walking_pattern(self, num_steps: int) -> dict:
        """Generate complete walking pattern"""
        
        # Generate time points
        total_time = num_steps * self.params.step_duration
        time_points = np.arange(0, total_time, self.params.control_dt)
        
        # Generate ZMP trajectory
        _, zmp_trajectory = self.pattern_generator.generate_zmp_trajectory(num_steps)
        
        # Generate CoM trajectory from ZMP
        com_trajectory = self.pattern_generator.generate_com_trajectory(zmp_trajectory, time_points)
        
        # Generate foot trajectories
        right_foot_traj, left_foot_traj = self.pattern_generator.generate_foot_trajectories(num_steps, time_points)
        
        # Generate joint angle trajectories (simplified - would need full IK)
        joint_trajectories = self.generate_joint_trajectories(right_foot_traj, left_foot_traj, com_trajectory)
        
        return {
            'time': time_points,
            'com_reference': com_trajectory,
            'zmp_reference': zmp_trajectory,
            'right_foot': right_foot_traj,
            'left_foot': left_foot_traj,
            'joint_angles': joint_trajectories,
            'body_orientation': np.zeros((len(time_points), 3))  # Keep upright
        }
    
    def generate_joint_trajectories(self, right_foot: np.ndarray, left_foot: np.ndarray, 
                                  com_trajectory: np.ndarray) -> np.ndarray:
        """Generate joint angle trajectories (simplified IK)"""
        num_points = len(com_trajectory)
        num_joints = 42  # HRP-4C has 42 DOF
        
        # This would normally involve full inverse kinematics
        # For now, return basic walking joint patterns
        joint_angles = np.zeros((num_points, num_joints))
        
        # Basic walking joint patterns (simplified)
        for i in range(num_points):
            # Hip joints (simplified sinusoidal patterns)
            phase = 2 * np.pi * i / (len(com_trajectory) / 2)  # Approximate step frequency
            
            # Right leg joints (indices 0-5: hip, knee, ankle)
            joint_angles[i, 1] = 0.3 * np.sin(phase)      # Right hip pitch
            joint_angles[i, 2] = 0.6 * max(0, np.sin(phase))  # Right knee pitch
            joint_angles[i, 3] = -0.3 * max(0, np.sin(phase)) # Right ankle pitch
            
            # Left leg joints (indices 6-11)
            joint_angles[i, 7] = 0.3 * np.sin(phase + np.pi)    # Left hip pitch
            joint_angles[i, 8] = 0.6 * max(0, np.sin(phase + np.pi))  # Left knee pitch
            joint_angles[i, 9] = -0.3 * max(0, np.sin(phase + np.pi)) # Left ankle pitch
            
        return joint_angles

class RealTimeWalkingController:
    """Real-time walking controller integrating pattern generation and LQR control"""
    
    def __init__(self, walking_system: CompleteWalkingSystem):
        self.walking_system = walking_system
        self.current_step = 0
        self.pattern_buffer = None
        self.time_index = 0
        
    def initialize_walking(self, num_steps: int):
        """Initialize walking pattern"""
        self.pattern_buffer = self.walking_system.generate_walking_pattern(num_steps)
        self.time_index = 0
        
    def control_step(self, measured_com: np.ndarray, measured_zmp: np.ndarray, 
                    foot_forces: np.ndarray) -> dict:
        """Execute one control step (5ms cycle)"""
        
        if self.pattern_buffer is None:
            raise ValueError("Walking pattern not initialized")
            
        if self.time_index >= len(self.pattern_buffer['time']):
            return None  # Walking completed
            
        # Get current references from pattern
        com_ref = self.pattern_buffer['com_reference'][self.time_index]
        zmp_ref = self.pattern_buffer['zmp_reference'][self.time_index]
        
        # Apply ZMP delay model
        actual_zmp = self.walking_system.zmp_delay_model.update(measured_zmp, 0.005)
        
        # LQR tracking control (from your implementation)
        com_error = measured_com - com_ref
        com_velocity = (measured_com - self.prev_com) / 0.005 if hasattr(self, 'prev_com') else np.zeros(2)
        zmp_error = actual_zmp - zmp_ref
        
        # State vector: [x, x_dot, p] for each axis
        state_error = np.array([com_error[0], com_velocity[0], zmp_error[0]])
        
        # LQR control law: u = K * state_error + reference
        zmp_modification_x = np.dot(self.walking_system.lqr_gains, state_error) + zmp_ref[0]
        
        # Repeat for y-axis
        state_error_y = np.array([com_error[1], com_velocity[1], zmp_error[1]])
        zmp_modification_y = np.dot(self.walking_system.lqr_gains, state_error_y) + zmp_ref[1]
        
        modified_zmp_ref = np.array([zmp_modification_x, zmp_modification_y])
        
        # Store for next iteration
        self.prev_com = measured_com.copy()
        self.time_index += 1
        
        return {
            'modified_zmp_reference': modified_zmp_ref,
            'joint_angle_reference': self.pattern_buffer['joint_angles'][self.time_index-1],
            'body_orientation_reference': self.pattern_buffer['body_orientation'][self.time_index-1],
            'foot_references': {
                'right': self.pattern_buffer['right_foot'][self.time_index-1],
                'left': self.pattern_buffer['left_foot'][self.time_index-1]
            }
        }


# if __name__ == "__main__":
#     import matplotlib.pyplot as plt

#     # Create walking system
#     params = WalkingParameters()
#     walking_system = CompleteWalkingSystem(params)

#     # Generate pattern for, say, 6 steps
#     num_steps = 6
#     pattern = walking_system.generate_walking_pattern(num_steps)

#     time = pattern['time']
#     com = pattern['com_reference']
#     zmp = pattern['zmp_reference']
#     right_foot = pattern['right_foot']
#     left_foot = pattern['left_foot']

#     # --- Plot CoM vs ZMP ---
#     plt.figure(figsize=(10, 5))
#     plt.subplot(2, 1, 1)
#     plt.plot(time, com[:, 0], label="CoM X")
#     plt.plot(time, zmp[:, 0], label="ZMP X", linestyle='--')
#     plt.ylabel("X Position (m)")
#     plt.legend()
#     plt.grid()

#     plt.subplot(2, 1, 2)
#     plt.plot(time, com[:, 1], label="CoM Y")
#     plt.plot(time, zmp[:, 1], label="ZMP Y", linestyle='--')
#     plt.xlabel("Time (s)")
#     plt.ylabel("Y Position (m)")
#     plt.legend()
#     plt.grid()
#     plt.suptitle("CoM vs ZMP Trajectories")

#     # --- Plot foot trajectories ---
#     plt.figure(figsize=(10, 5))
#     plt.plot(time, right_foot[:, 0], label="Right Foot X")
#     plt.plot(time, left_foot[:, 0], label="Left Foot X")
#     plt.plot(time, right_foot[:, 2], label="Right Foot Z")
#     plt.plot(time, left_foot[:, 2], label="Left Foot Z")
#     plt.xlabel("Time (s)")
#     plt.ylabel("Position (m)")
#     plt.legend()
#     plt.grid()
#     plt.title("Foot Trajectories (Swing and Support)")

#     plt.show()
