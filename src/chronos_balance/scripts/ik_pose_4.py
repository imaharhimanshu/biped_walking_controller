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
target_h = 0.23
target_shift = 0.0

# =========================================
# TERMINAL INPUT THREAD
# =========================================
def input_thread():

    global target_h, target_shift

    rospy.sleep(2.0)

    while not rospy.is_shutdown():

        try:
            print("\n==========================")
            print("Current Targets")
            print("==========================")
            print(f"Height : {target_h}")
            print(f"Shift  : {target_shift}")
            print("==========================")

            h = float(input("Enter height (0.18 - 0.23): "))
            s = float(input("Enter shift (-0.12 to 0.12): "))

            target_h = h
            target_shift = s

        except:
            print("Invalid Input")

# =========================================
# IK
# =========================================
def compute_ik(height):

    y = height
    x = 0.0

    d = math.sqrt(x*x + y*y)

    max_len = L1 + L2 - 0.00001
    d = min(d, max_len)

    cos_knee = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_knee = max(min(cos_knee, 1.0), -1.0)

    knee = math.acos(cos_knee)

    hip = math.atan2(x, y) - math.atan2(
        L2 * math.sin(knee),
        L1 + L2 * math.cos(knee)
    )

    ankle = -(hip + knee)

    return hip, knee, ankle

# =========================================
# MAIN
# =========================================
def main():

    global target_h, target_shift

    rospy.init_node("humanoid_crouch_shift")

    # =====================================
    # RIGHT LEG
    # =====================================
    right_hip_pub = rospy.Publisher(
        "/motor_upper_leg_below_v1_Revolute_31_position_controller/command",
        Float64,
        queue_size=1
    )

    right_knee_pub = rospy.Publisher(
        "/leg_knee_motor_a_Revolute_74_position_controller/command",
        Float64,
        queue_size=1
    )

    right_ankle_pub = rospy.Publisher(
        "/lower_leg_right_v1_Revolute_69_position_controller/command",
        Float64,
        queue_size=1
    )

    right_hip_roll_pub = rospy.Publisher(
        "/motor_upper_b_Rigid_131_position_controller/command",
        Float64,
        queue_size=1
    )

    right_ankle_roll_pub = rospy.Publisher(
        "/ankle_joint_ring1_a_Revolute_119_position_controller/command",
        Float64,
        queue_size=1
    )

    # =====================================
    # LEFT LEG
    # =====================================
    left_hip_pub = rospy.Publisher(
        "/motor_upper_leg_below_a_v1_Revolute_123_position_controller/command",
        Float64,
        queue_size=1
    )

    left_knee_pub = rospy.Publisher(
        "/leg_knee_motor_v1_Revolute_82_position_controller/command",
        Float64,
        queue_size=1
    )

    left_ankle_pub = rospy.Publisher(
        "/Lower_Leg_v2_v1_Revolute_91_position_controller/command",
        Float64,
        queue_size=1
    )

    left_hip_roll_pub = rospy.Publisher(
        "/motor_upper_a_Rigid_35_position_controller/command",
        Float64,
        queue_size=1
    )

    left_ankle_roll_pub = rospy.Publisher(
        "/ankle_joint_ring1_b_Revolute_115_position_controller/command",
        Float64,
        queue_size=1
    )

    # =====================================
    # START INPUT THREAD
    # =====================================
    threading.Thread(target=input_thread, daemon=True).start()

    # =====================================
    # CURRENT STATES
    # =====================================
    current_h = 0.23
    current_shift = 0.0

    rate = rospy.Rate(100)

    rospy.loginfo("Crouch + Shift Controller Running")

    while not rospy.is_shutdown():

        # =================================
        # SMOOTHING
        # =================================
        alpha = 0.02

        current_h += (target_h - current_h) * alpha
        current_shift += (target_shift - current_shift) * alpha

        # =================================
        # IK
        # =================================
        lh_hip, lh_knee, lh_ankle = compute_ik(current_h)
        rh_hip, rh_knee, rh_ankle = compute_ik(current_h)

        # =================================
        # PITCH
        # =================================
        rh = abs(rh_hip)
        rk = -abs(rh_knee)  # Right knee needs NEGATIVE angle to bend backwards
        ra = -abs(rh_ankle)

        lh = -abs(lh_hip)
        lk = abs(lh_knee)   # Left knee needs POSITIVE angle to bend backwards
        la = -abs(lh_ankle)

        # =================================
        # ROLL
        # =================================

        # Hip roll
        left_hip_roll = current_shift
        right_hip_roll = current_shift

        # Ankle compensation
        left_ankle_roll = -current_shift
        right_ankle_roll = -current_shift

        # =================================
        # PUBLISH RIGHT LEG
        # =================================
        right_hip_pub.publish(rh)
        right_knee_pub.publish(rk)
        right_ankle_pub.publish(ra)

        right_hip_roll_pub.publish(right_hip_roll)
        right_ankle_roll_pub.publish(right_ankle_roll)

        # =================================
        # PUBLISH LEFT LEG
        # =================================
        left_hip_pub.publish(lh)
        left_knee_pub.publish(lk)
        left_ankle_pub.publish(la)

        left_hip_roll_pub.publish(left_hip_roll)
        left_ankle_roll_pub.publish(left_ankle_roll)

        rate.sleep()

if __name__ == "__main__":
    main()
