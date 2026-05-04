#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybota002409/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)


        # --- Tunable Parameters ---

        # Goal distance to maintain from the tag (metres)
        self.GOAL_DISTANCE = 0.35

        # Angular PID gains (controls left/right rotation to centre the tag)
        self.ang_Kp = 3.0   # Proportional: main turning force
        self.ang_Ki = 0.0   # Integral: corrects persistent offset (start at 0)
        self.ang_Kd = 0.3   # Derivative: damps oscillation/overshoot

        # Linear PID gains (controls forward/backward to hold goal distance)
        self.lin_Kp = 0.6   # Proportional: main driving force
        self.lin_Ki = 0.0   # Integral: corrects if robot never quite reaches distance
        self.lin_Kd = 0.1   # Derivative: slows approach as distance closes

        # Speed limits
        self.MAX_OMEGA = 3.0    # Max angular velocity (rad/s)
        self.MIN_OMEGA = 0.5    # Min to overcome friction (rad/s)
        self.MAX_V     = 0.3    # Max linear velocity (m/s) — keep low for safety
        self.MIN_V     = 0.05   # Min linear velocity to overcome friction
        self.ANG_DEADZONE = 0.05  # x offset (m) within which we consider tag centred
        self.LIN_DEADZONE = 0.03  # distance error (m) within which we hold position

        # PID state
        self.ang_error_prev  = 0.0
        self.ang_error_sum   = 0.0
        self.lin_error_prev  = 0.0
        self.lin_error_sum   = 0.0
        self.last_time       = rospy.Time.now()

        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def apply_limits(self, value, min_val, max_val):
        """Clamp to max, then apply minimum threshold to overcome friction."""
        if abs(value) < 0.001:
            return 0.0  # Truly zero — don't apply minimum
        value = max(-max_val, min(max_val, value))
        if 0 < value < min_val:
            value = min_val
        elif -min_val < value < 0:
            value = -min_val
        return value

    def move_robot(self, detections):

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # Avoid division by zero on first callback or very fast callbacks
        if dt <= 0.0:
            dt = 0.01

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = now

        # ---- No tag: stay still (credit task — no seek required) ----
        if len(detections) == 0:
            rospy.loginfo("No tag detected. Holding position.")
            # Reset PID state so there's no windup when tag reappears
            self.ang_error_prev = 0.0
            self.ang_error_sum  = 0.0
            self.lin_error_prev = 0.0
            self.lin_error_sum  = 0.0
            self.stop_robot()
            return

        # ---- Read tag position ----
        x = detections[0].transform.translation.x  # left/right offset
        z = detections[0].transform.translation.z  # distance from camera
        rospy.loginfo("Tag  x=%.3f  z=%.3f  goal_z=%.3f", x, z, self.GOAL_DISTANCE)

        # CONTROL LOOP 1 — Angular (keep tag centred left/right)
        # error = x offset; goal is x = 0 (tag in centre)
        ang_error = x  # positive = tag left of centre → turn left

        ang_error_deriv      = (ang_error - self.ang_error_prev) / dt
        self.ang_error_sum  += ang_error * dt
        self.ang_error_prev  = ang_error

        omega_raw = (self.ang_Kp * ang_error +
                     self.ang_Ki * self.ang_error_sum +
                     self.ang_Kd * ang_error_deriv)

        if abs(ang_error) < self.ANG_DEADZONE:
            omega = 0.0
            self.ang_error_sum = 0.0  # Reset integral in deadzone
        else:
            omega = self.apply_limits(omega_raw, self.MIN_OMEGA, self.MAX_OMEGA)

        # CONTROL LOOP 2 — Linear (keep goal distance using z)
        # error = z - GOAL_DISTANCE; positive = too far → drive forward
        lin_error = z - self.GOAL_DISTANCE  # positive = too far away

        lin_error_deriv      = (lin_error - self.lin_error_prev) / dt
        self.lin_error_sum  += lin_error * dt
        self.lin_error_prev  = lin_error

        v_raw = (self.lin_Kp * lin_error +
                 self.lin_Ki * self.lin_error_sum +
                 self.lin_Kd * lin_error_deriv)

        if abs(lin_error) < self.LIN_DEADZONE:
            v = 0.0
            self.lin_error_sum = 0.0  # Reset integral in deadzone
        else:
            v = self.apply_limits(v_raw, self.MIN_V, self.MAX_V)

        rospy.loginfo("PID out → v=%.3f  omega=%.3f", v, omega)

        cmd_msg.v     = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass