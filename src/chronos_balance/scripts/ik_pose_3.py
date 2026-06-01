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

target_height = 0.23
target_shift = 0.0

# =========================================
# USER INPUT
# =========================================

def input_thread():

    global target_height
    global target_shift

    while True:

        try:

            print("\nExamples:")
            print("0.23 0.00")
            print("0.20 0.00")
            print("0.23 0.10")
            print("0.23 -0.10")

            vals = input(
                "\nEnter: height shift : "
            ).split()

            h = float(vals[0])
            s = float(vals[1])

            target_height = h
            target_shift = s

            print(
                f"Height: {h}  Shift: {s}"
            )

        except:

            print("Invalid input")

# =========================================
# IK
# =========================================

def compute_ik(height):

    crouch = 0.23 - height

    x = 0.02 * crouch

    y = height

    d = math.sqrt(x*x + y*y)

    max_len = L1 + L2 - 0.00001

    d = min(d, max_len)

    cos_knee = (
        d*d - L1*L1 - L2*L2
    ) / (2 * L1 * L2)

    cos_knee = max(min(cos_knee, 1.0), -1.0)

    knee = math.acos(cos_knee)

    hip = math.atan2(x, y) - math.atan2(
        L2 * math.sin(knee),
        L1 + L2 * math.cos(knee)
    )

    ankle = -(hip + knee) - 0.02

    return hip, knee, ankle

# =========================================
# MAIN
# =========================================

def main():

    global target_height
    global target_shift

    rospy.init_node("humanoid_10dof")

    # =====================================
    # PITCH JOINTS
    # =====================================

    right_hip_pub = rospy.Publisher(
        "/motor_upper_leg_below_v1_Revolute_31_position_controller/command",
        Float64,
        queue_size=1
    )

    right_knee_pub = rospy.Publisher(
        "/leg_knee_motor_v1_Revolute_82_position_controller/command",
        Float64,
        queue_size=1
    )

    right_ankle_pub = rospy.Publisher(
        "/lower_leg_right_v1_Revolute_69_position_controller/command",
        Float64,
        queue_size=1
    )

    left_hip_pub = rospy.Publisher(
        "/motor_upper_leg_below_a_v1_Revolute_123_position_controller/command",
        Float64,
        queue_size=1
    )

    left_knee_pub = rospy.Publisher(
        "/leg_knee_motor_a_Revolute_74_position_controller/command",
        Float64,
        queue_size=1
    )

    left_ankle_pub = rospy.Publisher(
        "/Lower_Leg_v2_v1_Revolute_91_position_controller/command",
        Float64,
        queue_size=1
    )

    # =====================================
    # HIP ROLL JOINTS
    # =====================================

    right_roll_pub = rospy.Publisher(
        "/motor_upper_b_Rigid-131_position_controller/command",
        Float64,
        queue_size=1
    )

    left_roll_pub = rospy.Publisher(
        "/motor_upper_a_Rigid-35_position_controller/command",
        Float64,
        queue_size=1
    )

    rospy.sleep(1)

    # =====================================
    # INPUT THREAD
    # =====================================

    threading.Thread(
        target=input_thread,
        daemon=True
    ).start()

    # =====================================
    # STATES
    # =====================================

    current_height = 0.23
    current_shift = 0.0

    rate = rospy.Rate(100)

    rospy.loginfo("10DOF humanoid IK running")

    while not rospy.is_shutdown():

        # =================================
        # SMOOTHING
        # =================================

        alpha = 0.01

        current_height += (
            target_height - current_height
        ) * alpha

        current_shift += (
            target_shift - current_shift
        ) * alpha

        # =================================
        # IK
        # =================================

        hip, knee, ankle = compute_ik(
            current_height
        )

        # =================================
        # PITCH
        # =================================

        rh = abs(hip)
        rk = abs(knee)
        ra = -abs(ankle)

        lh = -abs(hip)
        lk = -abs(knee)
        la = -abs(ankle)

        # =================================
        # HIP ROLL
        # =================================

        left_roll = current_shift
        right_roll = current_shift

        # =================================
        # PUBLISH
        # =================================

        right_hip_pub.publish(rh)
        right_knee_pub.publish(rk)
        right_ankle_pub.publish(ra)

        left_hip_pub.publish(lh)
        left_knee_pub.publish(lk)
        left_ankle_pub.publish(la)

        right_roll_pub.publish(right_roll)
        left_roll_pub.publish(left_roll)

        rate.sleep()

if __name__ == "__main__":
    main()
