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
# TARGETS (Shared State)
# =========================================
target_h_left = 0.22
target_h_right = 0.22
target_shift_left = 0.0
target_shift_right = 0.0
target_x_left = 0.0
target_x_right = 0.0

# =========================================
# SYSTEMATIC 6-STAGE GAIT STATE MACHINE
# =========================================
def set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, delay):
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    target_h_left = h_l
    target_h_right = h_r
    target_shift_left = shift_l
    target_shift_right = shift_r
    target_x_left = x_l
    target_x_right = x_r
    rospy.sleep(delay)

def swing_trajectory(duration):
    """
    Generates a continuous 50-point mathematical trajectory for the swing phase.
    Forces the IK solver to follow a perfect UP -> FORWARD -> DOWN arc.
    """
    steps = 50
    dt = duration / steps
    for i in range(steps + 1):
        t = i / float(steps)
        
        # 1. Forward Progression (x_r)
        # Swings from 0.00 to 0.15
        x_r = 0.00 + t * 0.15
        
        # 2. Sagittal Counterbalance (x_l)
        # Pelvis leans back from 0.00 to -0.03 to balance the swinging leg, keeping torso upright!
        x_l = 0.00 + t * (-0.03)
        
        # 3. Foot Height Arc (h_r)
        # Linear drop from 0.16 to 0.22, minus a massive sine wave bump for extra clearance!
        # At t=0.5, h_bump is -0.06, creating a peak lift height of 0.13m to force hip flexion!
        h_linear = 0.16 + t * (0.22 - 0.16)
        h_bump = -0.06 * math.sin(math.pi * t)
        h_r = h_linear + h_bump
        
        # 4. Lateral Stance (shift_r)
        # Smoothly interpolates from 0.20 to wide 0.18 landing stance
        shift_r = 0.20 + t * (0.18 - 0.20)
        
        # Set targets (h_l stays crouched at 0.22, shift_l stays at 0.25)
        set_targets(0.22, h_r, 0.25, shift_r, x_l, x_r, dt)

def return_to_standing(duration):
    """
    Smoothly interpolates all targets back to a neutral standing pose.
    """
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
    
    start_h_l = target_h_left
    start_h_r = target_h_right
    start_shift_l = target_shift_left
    start_shift_r = target_shift_right
    start_x_l = target_x_left
    start_x_r = target_x_right
    
    steps = 50
    dt = duration / steps if steps > 0 else 0
    
    for i in range(steps + 1):
        t = i / float(steps)
        # Ease-in, ease-out (cosine interpolation)
        ease_t = (1 - math.cos(t * math.pi)) / 2.0
        
        h_l = start_h_l + ease_t * (0.22 - start_h_l)
        h_r = start_h_r + ease_t * (0.22 - start_h_r)
        shift_l = start_shift_l + ease_t * (0.0 - start_shift_l)
        shift_r = start_shift_r + ease_t * (0.0 - start_shift_r)
        x_l = start_x_l + ease_t * (0.0 - start_x_l)
        x_r = start_x_r + ease_t * (0.0 - start_x_r)
        
        set_targets(h_l, h_r, shift_l, shift_r, x_l, x_r, dt)

def walk_sequence():
    try:
        rospy.loginfo("Resetting to standing pose...")
        return_to_standing(2.5)
        rospy.sleep(0.5)
        
        print("\n--- Starting Forward Step Sequence ---")
        
        print("Stage 1: Pre-Crouch (Double Support)...")
        # Crouch FIRST while both feet are firmly planted to safely lower CoM without falling backward!
        set_targets(0.22, 0.22, 0.0, 0.0, 0.0, 0.0, 3.0)
        
        print("Stage 2: Pure Lateral Shift (Reduced leftward mass)...")
        # Keep the safe crouch (0.20). Decrease lateral shift to 0.20.
        # Keep right leg PERFECTLY VERTICAL (0.00) to keep its heavy mass on the right side! (Stops leftward lean).
        set_targets(0.22, 0.22, 0.25, 0.25, 0.00, 0.00, 3) 
        
        print("Stage 3: Lifting Right Leg (Pre-emptive Forward Swing)...")
        # Lift Right Leg to 0.16. 
        # SWING FORWARD to 0.04 *during* the lift! This perfectly counteracts the right thigh pushing backward!
        set_targets(0.22, 0.16, 0.25, 0.20, 0.00, 0.00, 3)

        print("Stage 4: Continuous Sine-Wave Arc Trajectory...")
        # Execute the 50-step mathematical curve (UP -> FORWARD -> DOWN) with automatic torso counterbalance
        swing_trajectory(2.5)
        
        
        
        print("--- Step Complete! Biped Stable. ---\n")
    except rospy.ROSInterruptException:
        pass

def state_machine_thread():
    # Wait a moment for Gazebo to settle on startup
    rospy.sleep(2.5)
    
    while not rospy.is_shutdown():
        input("\n[PRESS ENTER] to execute a Static Step Forward...")
        walk_sequence()

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
# MAIN CONTROL LOOP
# =========================================
def main():
    global target_h_left, target_h_right, target_shift, target_x_left, target_x_right
    rospy.init_node("humanoid_ik_step")

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

    # Start the gait state machine thread
    threading.Thread(target=state_machine_thread, daemon=True).start()

    current_h_left = 0.22
    current_h_right = 0.22
    current_shift_left = 0.0
    current_shift_right = 0.0
    current_x_left = 0.0
    current_x_right = 0.0
    
    rate = rospy.Rate(100)

    rospy.loginfo("10DOF Humanoid Step Controller (ik_step.py) Running!")

    while not rospy.is_shutdown():
        # SMOOTH INTERPOLATION (Muscular damping)
        alpha = 0.030

        current_h_left += (target_h_left - current_h_left) * alpha
        current_h_right += (target_h_right - current_h_right) * alpha
        current_shift_left += (target_shift_left - current_shift_left) * alpha
        current_shift_right += (target_shift_right - current_shift_right) * alpha
        current_x_left += (target_x_left - current_x_left) * alpha
        current_x_right += (target_x_right - current_x_right) * alpha

        # IK CALCULATIONS
        lh_hip, lh_knee, lh_ankle = compute_ik(current_h_left, current_x_left)
        rh_hip, rh_knee, rh_ankle = compute_ik(current_h_right, current_x_right)

        # DYNAMIC ATAN2 SIGN AND PLANE MAPPINGS
        # Maps correct standing pose while unlocking true forward/backward sagittal motion
        rh_offset = 1.6 * math.atan2(current_x_right, current_h_right)
        rh = -rh_hip + rh_offset
        rk = -abs(rh_knee)  # Right knee needs NEGATIVE angle to bend backwards
        ra = rh_ankle + rh_offset  # Keep foot perfectly parallel to ground!

        lh_offset = -2 * math.atan2(current_x_left, current_h_left)
        lh = lh_hip + lh_offset
        lk = abs(lh_knee)   # Left knee needs POSITIVE angle to bend backwards
        la = lh_ankle - lh_offset  # Keep foot perfectly parallel to ground!
        
        # User requested: "rotate the left angle a little bit along the swing"
        # As the right leg swings forward, we gently lift the left heel (toe down) to push off.
        if current_x_right > 0.02:
            la += 0.8 * (current_x_right - 0.02)

        # ROLL CALCULATIONS (Leaning sideways)
        left_roll = current_shift_left
        right_roll = current_shift_right
        left_ankle_roll = -current_shift_left
        right_ankle_roll = -current_shift_right

        # PUBLISH JOINT TARGETS
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
