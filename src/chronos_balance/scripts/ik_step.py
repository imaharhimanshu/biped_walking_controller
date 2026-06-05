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
target_h_left = 0.23
target_h_right = 0.23
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
        # Swings to a safe 0.08m (8cm)
        x_r = 0.00 + t * 0.08
        
        # 2. Sagittal Forward Lean (x_l)
        # Because an 8cm step is massive, the pelvis MUST lean 5cm forward to keep the CoM balanced!
        x_l = 0.00 - t * 0.03
        
        # 3. Foot Height Arc (h_r)
        # Reduced the sine wave peak to 4cm (-0.04) for a natural, smooth swing!
        # Decreased final height to 0.20 so the leg stays bent, preventing early heel strike!
        h_linear = 0.16 + t * (0.19 - 0.16)
        h_bump = -0.04 * math.sin(math.pi * t)
        h_r = h_linear + h_bump
        
        # 4. Lateral Stance (shift_r)
        # Maintained the stable 0.25 to prevent leftward catapulting
        shift_r = 0.25
        
        # Set targets (h_l crouches to 0.21 to safely support the -0.05 backward stretch!)
        set_targets(0.21, h_r, 0.25, shift_r, x_l, x_r, dt)

def walk_sequence():
    try:
        rospy.loginfo("Resetting to standing pose...")
        set_targets(0.23, 0.23, 0.0, 0.0, 0.0, 0.0, 3.0)
        
        print("\n--- Starting Forward Step Sequence ---")
        
        print("Stage 1: Pre-Crouch (Double Support)...")
        set_targets(0.22, 0.22, 0.0, 0.0, 0.0, 0.0, 3.0)
        
        print("Stage 2: Pure Lateral Shift (Reduced leftward mass)...")
        set_targets(0.22, 0.22, 0.25, 0.25, 0.00, 0.00, 3) 
        
        print("Stage 3: Lifting Right Leg (Pre-emptive Forward Swing)...")
        # Keep shift_r at 0.25 so the legs stay perfectly parallel!
        set_targets(0.21, 0.16, 0.25, 0.25, 0.00, 0.00, 3)

        print("Stage 4: Continuous Sine-Wave Arc Trajectory...")
        swing_trajectory(2.5)
        
        print("Stage 5: Wide-Stance Touchdown (Safe Landing)...")
        # Touchdown starts from the swing trajectory (0.20 height, 0.08 forward)!
        # x_l = -0.04 catches the backward fall by keeping the pelvis perfectly over the feet!
        # h_r drops to 0.21 to firmly plant the foot on the ground.
        set_targets(0.21, 0.21, 0.05, 0.05, -0.04, 0.08, 2.0)
        
        print("Stage 6: Forward Weight Transfer (Synchronized)...")
        # Center the pelvis perfectly between the feet!
        # Distance is 13cm (-0.05 to 0.08). Midpoint is 0.065.
        set_targets(0.21, 0.21, 0.00, 0.00, -0.065, 0.065, 2.0)

        print("Stage 7: Stabilize Stance...")
        set_targets(0.21, 0.21, 0.00, 0.00, -0.065, 0.065, 2.0)
        
        print("Stage 8: Step Complete (Hold Pose)...")
        set_targets(0.21, 0.21, 0.00, 0.00, -0.065, 0.065, 3.0)
        
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
    global target_h_left, target_h_right, target_shift_left, target_shift_right, target_x_left, target_x_right
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

    current_h_left = 0.23
    current_h_right = 0.23
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
        # THE ONLY CHANGE: Flat Foot Ankle Fix
        rh = -rh_hip + 2.0 * math.atan2(current_x_right, current_h_right)
        rk = -abs(rh_knee)  
        ra = rh + rk

        lh = lh_hip - 2.0 * math.atan2(current_x_left, current_h_left)
        lk = abs(lh_knee)   
        la = -(lh + lk)

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
