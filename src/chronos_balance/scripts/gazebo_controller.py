#!/usr/bin/env python3

"""
Complete LQR-Based Biped Walking Controller
Implementation based on "Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking"
by Kajita et al.
"""

import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import scipy.linalg
from enum import Enum

class SupportPhase(Enum):
    RIGHT_SUPPORT = 0
    LEFT_SUPPORT = 1
    DOUBLE_SUPPORT = 2

@dataclass
class RobotParameters:
    """HRP-4C Robot Parameters"""
    height: float = 1.58  # meters
    mass: float = 43.0    # kg
    num_dof: float = 42
    com_height: float = 0.87  # meters (zc from paper)
    hip_width: float = 0.2    # distance between hip joints
    foot_length: float = 0.24
    foot_width: float = 0.12
    
@dataclass 
class WalkingParameters:
    """Walking Pattern Parameters"""
    step_length: float = 0.265        # meters
    step_duration: float = 1.1        # seconds  
    double_support_ratio: float = 0.2 # 20% of step duration
    step_height: float = 0.05         # swing foot height
    control_dt: float = 0.005         # 5ms control cycle
    walking_speed: float = 0.867      # km/h
    
@dataclass
class LQRParameters:
    """LQR Controller Parameters from Paper"""
    zmp_delay_time_constant: float = 0.05  # Tp = 0.05s
    pole_1: float = -13.0  # First assigned pole
    pole_2: float = -3.0   # Second assigned pole  
    # Third pole is -omega_c (calculated dynamically)

class LinearInvertedPendulumModel:
    """Linear Inverted Pendulum Model with ZMP Delay"""
    
    def __init__(self, robot_params: RobotParameters, lqr_params: LQRParameters):
        self.robot_params = robot_params
        self.lqr_params = lqr_params
        self.gravity = 9.81
        
        # Calculate natural frequency omega_c = sqrt(g/zc)
        self.omega_c = math.sqrt(self.gravity / robot_params.com_height)
        
        # System matrices for state space representation
        # State: x = [com_pos, com_vel, zmp_actual]
        self.A = np.array([
            [0, 1, 0],
            [self.gravity/robot_params.com_height, 0, -self.gravity/robot_params.com_height],
            [0, 0, -1/lqr_params.zmp_delay_time_constant]
        ])
        
        self.B = np.array([
            [0],
            [0], 
            [1/lqr_params.zmp_delay_time_constant]
        ])
        
        # Calculate LQR gains using pole placement
        self.K = self._calculate_lqr_gains()
        
    def _calculate_lqr_gains(self) -> np.ndarray:
        """Calculate LQR gains using pole placement as specified in paper"""
        # Desired poles: [-13, -3, -omega_c]
        desired_poles = np.array([
            self.lqr_params.pole_1,
            self.lqr_params.pole_2, 
            -self.omega_c
        ])
        
        # Use pole placement to calculate feedback gains
        K = scipy.linalg.place_poles(self.A, self.B, desired_poles)
        return K.flatten()
    
    def get_control_input(self, state_error: np.ndarray, reference_zmp: float) -> float:
        """Calculate control input using LQR feedback"""
        # u = K * state_error + reference
        return np.dot(self.K, state_error) + reference_zmp

class ZMPTrajectoryGenerator:
    """Generates ZMP trajectories using cubic polynomials"""
    
    def __init__(self, walking_params: WalkingParameters, robot_params: RobotParameters):
        self.walking_params = walking_params
        self.robot_params = robot_params
        
    def generate_zmp_trajectory(self, num_steps: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate ZMP trajectory for sagittal and lateral directions"""
        total_time = num_steps * self.walking_params.step_duration
        time_points = np.arange(0, total_time, self.walking_params.control_dt)
        num_points = len(time_points)
        
        zmp_x = np.zeros(num_points)  # Sagittal (forward/backward)
        zmp_y = np.zeros(num_points)  # Lateral (left/right)
        support_phase = np.zeros(num_points, dtype=int)
        
        for i, t in enumerate(time_points):
            step_number = int(t / self.walking_params.step_duration)
            time_in_step = t - step_number * self.walking_params.step_duration
            step_progress = time_in_step / self.walking_params.step_duration
            
            # Determine support phase
            ds_threshold = self.walking_params.double_support_ratio
            
            if step_progress < ds_threshold/2 or step_progress > (1 - ds_threshold/2):
                support_phase[i] = SupportPhase.DOUBLE_SUPPORT.value
            else:
                if step_number % 2 == 0:
                    support_phase[i] = SupportPhase.RIGHT_SUPPORT.value
                else:
                    support_phase[i] = SupportPhase.LEFT_SUPPORT.value
            
            # Generate ZMP trajectory
            if support_phase[i] == SupportPhase.RIGHT_SUPPORT.value:
                zmp_y[i] = -self.robot_params.hip_width / 2  # Right foot
            elif support_phase[i] == SupportPhase.LEFT_SUPPORT.value:
                zmp_y[i] = self.robot_params.hip_width / 2   # Left foot
            else:  # Double support
                # Smooth transition between feet
                if step_progress < ds_threshold/2:
                    # Transitioning from previous step
                    prev_foot = -1 if (step_number-1) % 2 == 0 else 1
                    curr_foot = -1 if step_number % 2 == 0 else 1
                    blend = step_progress / (ds_threshold/2)
                    zmp_y[i] = prev_foot * self.robot_params.hip_width/2 * (1-blend) + \
                              curr_foot * self.robot_params.hip_width/2 * blend
                else:
                    # Transitioning to next step  
                    curr_foot = -1 if step_number % 2 == 0 else 1
                    next_foot = -1 if (step_number+1) % 2 == 0 else 1
                    blend = (step_progress - (1-ds_threshold/2)) / (ds_threshold/2)
                    zmp_y[i] = curr_foot * self.robot_params.hip_width/2 * (1-blend) + \
                              next_foot * self.robot_params.hip_width/2 * blend
            
            # Forward progression
            zmp_x[i] = step_number * self.walking_params.step_length + \
                      step_progress * self.walking_params.step_length * 0.1
        
        return time_points, np.column_stack([zmp_x, zmp_y]), support_phase

class CoMTrajectoryGenerator:
    """Generates CoM trajectories from ZMP using LIPM"""
    
    def __init__(self, lipm_model: LinearInvertedPendulumModel):
        self.lipm_model = lipm_model  # Fixed: was limp_model
        
    def generate_com_trajectory(self, zmp_trajectory: np.ndarray, 
                               time_points: np.ndarray) -> np.ndarray:
        """Generate CoM trajectory using LIMP dynamics"""
        num_points = len(time_points)
        com_trajectory = np.zeros((num_points, 2))
        com_velocity = np.zeros((num_points, 2))
        
        # Initial conditions
        com_trajectory[0] = zmp_trajectory[0]  # Fixed: added [0] index
        com_velocity[0] = np.array([0.0, 0.0])  # Fixed: added [0] index
        
        dt = time_points[1] - time_points[0]  # Fixed: added [0] index
        omega_sq = self.lipm_model.omega_c ** 2
        
        for i in range(1, num_points):
            for axis in range(2):  # x and y axes
                # LIPM dynamics: x_ddot = omega^2 * (x - p)
                acceleration = omega_sq * (com_trajectory[i-1, axis] - zmp_trajectory[i-1, axis])
                com_velocity[i, axis] = com_velocity[i-1, axis] + acceleration * dt
                com_trajectory[i, axis] = com_trajectory[i-1, axis] + com_velocity[i, axis] * dt
        
        return com_trajectory, com_velocity


class FootTrajectoryGenerator:
    """Generates foot trajectories for swing phases"""
    
    def __init__(self, walking_params: WalkingParameters, robot_params: RobotParameters):
        self.walking_params = walking_params
        self.robot_params = robot_params
        
    def generate_foot_trajectories(self, num_steps: int, time_points: np.ndarray, 
                                 support_phases: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Generate 6DOF trajectories for both feet"""
        num_points = len(time_points)
        
        # [x, y, z, roll, pitch, yaw]
        right_foot = np.zeros((num_points, 6))
        left_foot = np.zeros((num_points, 6))
        
        # Initialize foot positions
        right_foot[:, 1] = -self.robot_params.hip_width / 2  # Right foot y-position
        left_foot[:, 1] = self.robot_params.hip_width / 2    # Left foot y-position
        
        for i, t in enumerate(time_points):
            step_number = int(t / self.walking_params.step_duration)
            time_in_step = t - step_number * self.walking_params.step_duration
            step_progress = time_in_step / self.walking_params.step_duration
            
            # Determine which foot is swinging
            if support_phases[i] == SupportPhase.LEFT_SUPPORT.value:
                # Right foot swinging
                target_x = (step_number + 1) * self.walking_params.step_length
                swing_progress = self._get_swing_progress(step_progress)
                
                right_foot[i, 0] = right_foot[max(0, i-1), 0] + \
                                  (target_x - right_foot[max(0, i-1), 0]) * swing_progress
                right_foot[i, 2] = self.walking_params.step_height * \
                                  np.sin(np.pi * swing_progress)
                                  
            elif support_phases[i] == SupportPhase.RIGHT_SUPPORT.value:
                # Left foot swinging  
                target_x = (step_number + 1) * self.walking_params.step_length
                swing_progress = self._get_swing_progress(step_progress)
                
                left_foot[i, 0] = left_foot[max(0, i-1), 0] + \
                                 (target_x - left_foot[max(0, i-1), 0]) * swing_progress
                left_foot[i, 2] = self.walking_params.step_height * \
                                 np.sin(np.pi * swing_progress)
            else:
                # Double support - both feet on ground
                right_foot[i, 2] = 0.0
                left_foot[i, 2] = 0.0
        
        return right_foot, left_foot
    
    def _get_swing_progress(self, step_progress: float) -> float:
        """Calculate swing phase progress (0 to 1)"""
        ds_ratio = self.walking_params.double_support_ratio
        
        if step_progress <= ds_ratio/2:
            return 0.0
        elif step_progress >= 1 - ds_ratio/2:
            return 1.0
        else:
            # Linear progress during single support phase
            single_support_start = ds_ratio/2
            single_support_duration = 1 - ds_ratio
            return (step_progress - single_support_start) / single_support_duration

class PostureForceController:
    """Implements the middle layer posture/force control from the paper"""
    
    def __init__(self, robot_params: RobotParameters, lqr_params: LQRParameters):
        self.robot_params = robot_params
        self.lqr_params = lqr_params
        
        # Control gains from paper
        self.chest_posture_gain = 5.0
        self.chest_time_constant = 0.1
        self.damping_gain = 100.0
        self.force_time_constant = 0.05
        
    def chest_posture_control(self, current_orientation: np.ndarray, 
                            desired_orientation: np.ndarray, 
                            dt: float) -> np.ndarray:
        """Implement chest posture controller from equations (1) and (2)"""
        orientation_error = desired_orientation - current_orientation
        
        # First-order system with feedback
        # Δφ̇ = kC(φd - φ) - (1/TC)Δφ  
        orientation_rate = self.chest_posture_gain * orientation_error - \
                          (1/self.chest_time_constant) * orientation_error
        
        return orientation_rate * dt
    
    def zmp_force_distribution(self, desired_zmp: np.ndarray,
                              right_foot_pos: np.ndarray,
                              left_foot_pos: np.ndarray,
                              support_phase: SupportPhase) -> Tuple[np.ndarray, np.ndarray]:
        """Distribute ZMP into foot forces and torques"""
        total_weight = self.robot_params.mass * 9.81
        
        if support_phase == SupportPhase.RIGHT_SUPPORT:
            alpha = 1.0
        elif support_phase == SupportPhase.LEFT_SUPPORT:
            alpha = 0.0
        else:  # Double support
            # Calculate distribution ratio using heuristic from paper
            alpha = self._calculate_force_distribution_ratio(
                desired_zmp, right_foot_pos, left_foot_pos)
        
        # Force distribution (equations 4 and 5)
        right_force = np.array([0, 0, -alpha * total_weight])
        left_force = np.array([0, 0, -(1-alpha) * total_weight])
        
        # Torque calculation (equation 6)
        right_torque = np.cross(right_foot_pos - desired_zmp, right_force)
        left_torque = np.cross(left_foot_pos - desired_zmp, left_force)
        
        return (right_force, right_torque), (left_force, left_torque)
    
    def _calculate_force_distribution_ratio(self, zmp: np.ndarray,
                                          right_foot: np.ndarray, 
                                          left_foot: np.ndarray) -> float:
        """Calculate alpha using geometric heuristic from paper"""
        # Project ZMP onto line connecting feet
        foot_vector = left_foot - right_foot
        foot_length = np.linalg.norm(foot_vector[:2])
        
        if foot_length < 1e-6:
            return 0.5
        
        # Calculate projection coefficient
        zmp_vector = zmp - right_foot
        projection = np.dot(zmp_vector[:2], foot_vector[:2]) / (foot_length**2)
        
        # Clamp to [0, 1] range
        alpha = max(0.0, min(1.0, 1.0 - projection))
        return alpha
    
    def foot_torque_control(self, desired_torque: np.ndarray,
                           measured_torque: np.ndarray,
                           dt: float) -> np.ndarray:
        """Implement foot torque damping controller from equation (14)"""
        torque_error = desired_torque - measured_torque
        
        # δ̇ = D^(-1)(τd - τ) - (1/T)δ
        control_rate = torque_error / self.damping_gain - \
                      measured_torque / self.force_time_constant
        
        return control_rate * dt

class LQRWalkingController:
    """Main LQR-based walking controller integrating all components"""
    
    def __init__(self, robot_params: RobotParameters, 
                 walking_params: WalkingParameters,
                 lqr_params: LQRParameters):
        
        self.robot_params = robot_params
        self.walking_params = walking_params
        self.lqr_params = lqr_params
        
        # Initialize components
        self.lipm_model = LinearInvertedPendulumModel(robot_params, lqr_params)
        self.zmp_generator = ZMPTrajectoryGenerator(walking_params, robot_params)
        self.com_generator = CoMTrajectoryGenerator(self.lipm_model)  # Fixed: was self.limp_model
        self.foot_generator = FootTrajectoryGenerator(walking_params, robot_params)
        self.posture_controller = PostureForceController(robot_params, lqr_params)
        
        # State variables
        self.previous_com = np.zeros(2)
        self.previous_zmp = np.zeros(2)
        self.time_index = 0
        self.walking_pattern = None

        
    def generate_walking_pattern(self, num_steps: int) -> Dict:
        """Generate complete walking pattern with LQR-optimized trajectories"""
        
        # Generate base ZMP trajectory
        time_points, zmp_traj, support_phases = self.zmp_generator.generate_zmp_trajectory(num_steps)
        
        # Generate CoM trajectory from ZMP
        com_traj, com_vel_traj = self.com_generator.generate_com_trajectory(zmp_traj, time_points)
        
        # Generate foot trajectories
        right_foot_traj, left_foot_traj = self.foot_generator.generate_foot_trajectories(
            num_steps, time_points, support_phases)
        
        # Store pattern
        self.walking_pattern = {
            'time': time_points,
            'zmp_reference': zmp_traj,
            'com_reference': com_traj,
            'com_velocity_reference': com_vel_traj,
            'right_foot_reference': right_foot_traj,
            'left_foot_reference': left_foot_traj,
            'support_phases': support_phases,
            'body_orientation_reference': np.zeros((len(time_points), 3))
        }
        
        return self.walking_pattern
    
    def control_step(self, sensor_data: Dict) -> Optional[Dict]:
        """Execute one control cycle with LQR feedback"""
        
        if self.walking_pattern is None:
            raise ValueError("Walking pattern not generated")
            
        if self.time_index >= len(self.walking_pattern['time']):
            return None  # Walking completed
        
        # Get current references
        com_ref = self.walking_pattern['com_reference'][self.time_index]
        com_vel_ref = self.walking_pattern['com_velocity_reference'][self.time_index]
        zmp_ref = self.walking_pattern['zmp_reference'][self.time_index]
        
        # Get sensor measurements
        measured_com = sensor_data['com_position']
        measured_com_vel = sensor_data['com_velocity'] 
        measured_zmp = sensor_data['zmp_position']
        
        # Apply ZMP delay model (first-order filter)
        zmp_filter_gain = self.walking_params.control_dt / self.lqr_params.zmp_delay_time_constant
        actual_zmp = self.previous_zmp + zmp_filter_gain * (measured_zmp - self.previous_zmp)
        self.previous_zmp = actual_zmp.copy()
        
        # Calculate LQR control for each axis
        modified_zmp = np.zeros(2)
        
        for axis in range(2):  # x and y axes
        # State error vector: [com_pos_error, com_vel_error, zmp_error]
            state_error = np.array([
                measured_com[axis] - com_ref[axis],
                measured_com_vel[axis] - com_vel_ref[axis], 
                actual_zmp[axis] - zmp_ref[axis]
            ])

            # LQR control law: u = K * error + reference
            modified_zmp[axis] = self.lipm_model.get_control_input(  # Fixed: was self.limp_model
                state_error, zmp_ref[axis])
        
        # Posture control
        current_orientation = sensor_data['body_orientation']
        desired_orientation = self.walking_pattern['body_orientation_reference'][self.time_index]
        orientation_correction = self.posture_controller.chest_posture_control(
            current_orientation, desired_orientation, self.walking_params.control_dt)
        
        # Force distribution
        support_phase = SupportPhase(self.walking_pattern['support_phases'][self.time_index])
        right_foot_pos = self.walking_pattern['right_foot_reference'][self.time_index][:3]
        left_foot_pos = self.walking_pattern['left_foot_reference'][self.time_index][:3]
        
        (right_force, right_torque), (left_force, left_torque) = \
            self.posture_controller.zmp_force_distribution(
                np.append(modified_zmp, 0), right_foot_pos, left_foot_pos, support_phase)
        
        # Update state
        self.time_index += 1
        
        # Return control commands
        return {
            'modified_zmp_reference': modified_zmp,
            'right_foot_reference': self.walking_pattern['right_foot_reference'][self.time_index-1],
            'left_foot_reference': self.walking_pattern['left_foot_reference'][self.time_index-1],
            'body_orientation_reference': desired_orientation + orientation_correction,
            'right_foot_force': right_force,
            'right_foot_torque': right_torque,
            'left_foot_force': left_force,
            'left_foot_torque': left_torque,
            'support_phase': support_phase,
            'debug_info': {
                'com_error': measured_com - com_ref,
                'zmp_error': actual_zmp - zmp_ref,
                'lqr_gains': self.limp_model.K
            }
        }
    
    def reset_controller(self):
        """Reset controller state"""
        self.previous_com = np.zeros(2)
        self.previous_zmp = np.zeros(2) 
        self.time_index = 0
        self.walking_pattern = None

# Usage Example and Main Interface
class HumanoidWalkingSystem:
    """Complete humanoid walking system interface"""
    
    def __init__(self):
        # Initialize with HRP-4C parameters from paper
        self.robot_params = RobotParameters()
        self.walking_params = WalkingParameters()
        self.lqr_params = LQRParameters()
        
        self.controller = LQRWalkingController(
            self.robot_params, self.walking_params, self.lqr_params)
    
    def start_walking(self, num_steps: int = 10):
        """Initialize and start walking"""
        print(f"Generating walking pattern for {num_steps} steps...")
        pattern = self.controller.generate_walking_pattern(num_steps)
        
        print(f"Pattern generated with {len(pattern['time'])} time points")
        print(f"Walking duration: {pattern['time'][-1]:.2f} seconds")
        print(f"LQR gains: {self.controller.limp_model.K}")
        
        return pattern
    
    def execute_control_cycle(self, sensor_data: Dict) -> Optional[Dict]:
        """Execute one control cycle"""
        return self.controller.control_step(sensor_data)
    
    def get_system_parameters(self) -> Dict:
        """Get current system parameters"""
        return {
            'robot': self.robot_params,
            'walking': self.walking_params,
            'lqr': self.lqr_params,
            'omega_c': self.controller.limp_model.omega_c,
            'lqr_gains': self.controller.limp_model.K
        }

if __name__ == "__main__":
    # Example usage
    walking_system = HumanoidWalkingSystem()
    
    # Generate walking pattern
    pattern = walking_system.start_walking(num_steps=5)
    
    # Simulate control loop
    print("\nSimulating walking control loop...")
    
    for i in range(min(100, len(pattern['time']))):  # Simulate first 100 steps
        
        # Simulate sensor data (in real system, this would come from robot sensors)
        sensor_data = {
            'com_position': pattern['com_reference'][i] + np.random.normal(0, 0.001, 2),
            'com_velocity': pattern['com_velocity_reference'][i] + np.random.normal(0, 0.01, 2),
            'zmp_position': pattern['zmp_reference'][i] + np.random.normal(0, 0.002, 2),
            'body_orientation': np.random.normal(0, 0.01, 3)
        }
        
        # Execute control step
        control_output = walking_system.execute_control_cycle(sensor_data)
        
        if control_output is None:
            print("Walking completed!")
            break
            
        # Print debug info every 50 steps
        if i % 50 == 0:
            print(f"Step {i}: CoM error = {control_output['debug_info']['com_error']}")
            print(f"         ZMP error = {control_output['debug_info']['zmp_error']}")
    
    print("\nWalking simulation completed successfully!")
