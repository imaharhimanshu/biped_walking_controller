#!/usr/bin/env python3

import rospy
import math
import threading
from std_msgs.msg import Float64

# =========================================
# LEG LENGTHS
# =========================================
L1 = 0.11
L2 = 0.116

# =========================================
# TARGETS 
# =========================================
target_h_left = 0.23
target_h_right = 0.23
target_shift = 0.0
target_x_left = 0.0
target_x_right = 0.0

# =========================================
# THE STATE MACHINE
# =========================================
def set_targets(h_l, h_r, shift, x_l, x_r, delay):
    global target_h_left, target_h_right, target_shift, target_x_left, target_x_right
    target_h_left = h_l
    target_h_right = h_r
    target_shift = shift
    target_x_left = x_l
    target_x_right = x_r
    rospy.sleep(delay)

def state_machine_thread():
    # Wait a moment for Gazebo to settle on startup
    rospy.sleep(2.0)
    
    while not rospy.is_shutdown():
        input("\n[PRESS ENTER] to execute a Static Step Forward...")
        
        print("State 1: Pure Lateral Shift (Moving weight to left leg)...")
        # ONLY shift sideways to 0.25. No forward/backward bracing yet.
        # H_L | H_R | Shift | X_L | X_R | Delay
        set_targets(0.23, 0.23, 0.25, 0.00, 0.00, 3.0) 
        
        print("State 2: Pure Pitch Brace (Leaning torso slightly back)...")
        # Set foot offsets to -0.01 to physically lean the hips/torso BACKWARDS
        set_targets(0.23, 0.23, 0.25, -0.01, -0.01, 2.0)
        
        print("State 3: Lifting right leg...")
        # Lift the right leg straight up while keeping torso braced backward (delay 3.0s)
        set_targets(0.25, 0.15, 0.25, -0.01, -0.01, 3.0) 
        
        print("State 4: Swinging right leg forward...")
        # Swing right foot to 0.06 (forward) while Left Leg rolls torso forward (x_l = -0.03) and remains fully extended (0.25)
        set_targets(0.25, 0.18, 0.25, -0.03, 0.06, 2.0)
        
        print("State 5: Planting right foot & Centering lateral shift (Synchronized)...")
        # Extend right leg to 0.23, reduce shift to 0.00, and keep torso rolled forward (x_l = -0.03) for flat touchdown
        set_targets(0.23, 0.23, 0.00, -0.03, 0.06, 3.0) 
        
        print("State 6: Recovering Stance (Centering foot offsets)...")
        # Smoothly recover foot offsets from split stance back to 0.00 stance
        set_targets(0.23, 0.23, 0.00, 0.00, 0.00, 2.0) 
        
        print("--- Step Complete! ---")
# =========================================
# INVERSE KINEMATICS
# =========================================
def compute_ik(height, x_offset):
    y = height
    x = x_offset
    d = math.sqrt(x*x + y*y)
    
    max_len = L1 + L2 - 0.00001
    d = min(d, max_len)

    cos_knee = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_knee = max(min(cos_knee, 1.0), -1.0)
    knee = math.acos(cos_knee)

    hip = math.atan2(x, y) - math.atan2(L2 * math.sin(knee), L1 + L2 * math.cos(knee))
    ankle = -(hip + knee) 

    return hip, knee, ankle

# =========================================
# MAIN
# =========================================
def main():
    global target_h_left, target_h_right, target_shift, target_x_left, target_x_right
    rospy.init_node("humanoid_10dof")

    # Right Leg Publishers
    right_hip_pub = rospy.Publisher("/motor_upper_leg_below_v1_Revolute_31_position_controller/command", Float64, queue_size=1)
    right_knee_pub = rospy.Publisher("/leg_knee_motor_a_Revolute_74_position_controller/command", Float64, queue_size=1) 
    right_ankle_pub = rospy.Publisher("/lower_leg_right_v1_Revolute_69_position_controller/command", Float64, queue_size=1)
    right_hip_roll_pub = rospy.Publisher("/motor_upper_b_Rigid_131_position_controller/command", Float64, queue_size=1)
    right_ankle_roll_pub = rospy.Publisher("/ankle_joint_ring1_a_Revolute_119_position_controller/command", Float64, queue_size=1) 

    # Left Leg Publishers
    left_hip_pub = rospy.Publisher("/motor_upper_leg_below_a_v1_Revolute_123_position_controller/command", Float64, queue_size=1)
    left_knee_pub = rospy.Publisher("/leg_knee_motor_v1_Revolute_82_position_controller/command", Float64, queue_size=1) 
    left_ankle_pub = rospy.Publisher("/Lower_Leg_v2_v1_Revolute_91_position_controller/command", Float64, queue_size=1)
    left_hip_roll_pub = rospy.Publisher("/motor_upper_a_Rigid_35_position_controller/command", Float64, queue_size=1)
    left_ankle_roll_pub = rospy.Publisher("/ankle_joint_ring1_b_Revolute_115_position_controller/command", Float64, queue_size=1) 

    # Start the state machine in the background
    threading.Thread(target=state_machine_thread, daemon=True).start()

    current_h_left = 0.23
    current_h_right = 0.23
    current_shift = 0.0
    current_x_left = 0.0
    current_x_right = 0.0
    rate = rospy.Rate(100)

    rospy.loginfo("10DOF Humanoid State Machine Running!")

    while not rospy.is_shutdown():
        # SMOOTHING (The "Muscles")
        alpha = 0.02

        current_h_left += (target_h_left - current_h_left) * alpha
        current_h_right += (target_h_right - current_h_right) * alpha
        current_shift += (target_shift - current_shift) * alpha
        current_x_left += (target_x_left - current_x_left) * alpha
        current_x_right += (target_x_right - current_x_right) * alpha

        # IK
        lh_hip, lh_knee, lh_ankle = compute_ik(current_h_left, current_x_left)
        rh_hip, rh_knee, rh_ankle = compute_ik(current_h_right, current_x_right)

        rh = abs(rh_hip)
        rk = -abs(rh_knee)  # Right knee needs NEGATIVE angle to bend backwards
        ra = -abs(rh_ankle)

        lh = -abs(lh_hip)
        lk = abs(lh_knee)   # Left knee needs POSITIVE angle to bend backwards
        la = -abs(lh_ankle)

        # ROLL
        left_roll = current_shift
        right_roll = current_shift
        left_ankle_roll = -current_shift
        right_ankle_roll = -current_shift

        # PUBLISH
        right_hip_pub.publish(rh)
        right_knee_pub.publish(rk)
        right_ankle_pub.publish(ra)
        right_hip_roll_pub.publish(right_roll)
        right_ankle_roll_pub.publish(right_ankle_roll)

        left_hip_pub.publish(lh)
        left_knee_pub.publish(lk)
        left_ankle_pub.publish(la)
        left_hip_roll_pub.publish(left_roll)
        left_ankle_roll_pub.publish(left_ankle_roll)

        rate.sleep()

if __name__ == "__main__":
    main()
