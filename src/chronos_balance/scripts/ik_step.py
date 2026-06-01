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
target_shift = 0.0
target_x_left = 0.0
target_x_right = 0.0

# =========================================
# SYSTEMATIC 6-STAGE GAIT STATE MACHINE
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
    rospy.sleep(2.5)
    
    while not rospy.is_shutdown():
        input("\n[PRESS ENTER] to execute a Static Step Forward...")
        
        print("\n--- Starting Forward Step Sequence ---")
        
        print("Stage 1: Pure Lateral Shift (Tilting weight over Left Leg)...")
        # Tilt robot sideways to 0.25 to shift CoM over Left foot. Keep feet flat.
        set_targets(0.23, 0.23, 0.25, 0.00, 0.00, 3.0) 
        
        print("Stage 2: Lifting Right Leg (Straight up, no forward lean)...")
        # Lift Right Leg by shortening it to 0.16 (7cm lift). Keep supporting leg fully extended.
        # This is the most stable posture to break contact with the floor.
        set_targets(0.25, 0.16, 0.25, 0.00, 0.00, 2.5)
        
        print("Stage 3: Swinging & Knee Extension (Right leg swings in the air, Torso stays braced BACK)...")
        # Swing right foot to 0.07, extend the right knee (h_r=0.20) so it reaches forward.
        # Keep Left Leg (supporting) braced backward (x_l = -0.01) to keep the torso back and counterbalance the swinging leg!
        # Use optimal 0.19 lateral shift to prevent over-leaning and tripping.
        set_targets(0.25, 0.20, 0.1615, -0.01, 0.07, 2.5)
        
        print("Stage 4: Landing & Torso Roll (Left leg rolls forward as Right foot plants, knees bent for compliance)...")
        # Lower pelvis to 0.20m (deep crouch) to ensure Right leg (x_r=0.07) physically reaches the ground!
        # Torso rolls forward (x_l = -0.03) and shift remains at 0.19 for single-leg balance.
        set_targets(0.20, 0.20, 0.1615, -0.03, 0.07, 2.5)
        
        print("Stage 5: Weight Transfer (Fully centering lateral tilt in deep crouch)...")
        # Center lateral shift to 0.00 while keeping pelvis low (0.20m) for stable double-support weight transfer.
        set_targets(0.20, 0.20, 0.00, -0.03, 0.07, 2.0)
        
        print("Stage 6: Recovering Stance (Centering foot offsets and standing tall)...")
        # Recover foot offsets to 0.00 and stand tall by straightening both knees to 0.23m standing height.
        set_targets(0.23, 0.23, 0.00, 0.00, 0.00, 2.0)
        
        print("--- Step Complete! Biped Stable. ---\n")

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

    current_h_left = 0.23
    current_h_right = 0.23
    current_shift = 0.0
    current_x_left = 0.0
    current_x_right = 0.0
    
    rate = rospy.Rate(100)

    rospy.loginfo("10DOF Humanoid Step Controller (ik_step.py) Running!")

    while not rospy.is_shutdown():
        # SMOOTH INTERPOLATION (Muscular damping)
        alpha = 0.02

        current_h_left += (target_h_left - current_h_left) * alpha
        current_h_right += (target_h_right - current_h_right) * alpha
        current_shift += (target_shift - current_shift) * alpha
        current_x_left += (target_x_left - current_x_left) * alpha
        current_x_right += (target_x_right - current_x_right) * alpha

        # IK CALCULATIONS
        lh_hip, lh_knee, lh_ankle = compute_ik(current_h_left, current_x_left)
        rh_hip, rh_knee, rh_ankle = compute_ik(current_h_right, current_x_right)

        # DYNAMIC ATAN2 SIGN AND PLANE MAPPINGS
        # Maps correct standing pose while unlocking true forward/backward sagittal motion
        rh = -rh_hip + 2 * math.atan2(current_x_right, current_h_right)
        rk = -abs(rh_knee)  # Right knee needs NEGATIVE angle to bend backwards
        ra = rh_ankle

        lh = lh_hip - 2 * math.atan2(current_x_left, current_h_left)
        lk = abs(lh_knee)   # Left knee needs POSITIVE angle to bend backwards
        la = lh_ankle

        # ROLL CALCULATIONS (Leaning sideways)
        left_roll = current_shift
        right_roll = current_shift
        left_ankle_roll = -current_shift
        right_ankle_roll = -current_shift

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
