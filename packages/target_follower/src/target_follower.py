#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybota002409/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################


        self.tag_detected = False 

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):

        #### YOUR CODE GOES HERE ####
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0 

        # ---- Feature 1: Seek behavior (no tag visible) ----
        if len(detections) == 0:
            if self.tag_detected:
                rospy.loginfo("Tag lost. Returning to seek behavior...")
            self.tag_detected = False

            # Rotate slowly to scan for a tag
            SEEK_SPEED = 2.5
            cmd_msg.omega = SEEK_SPEED
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # ---- Feature 2: Look at object (tag visible) ----
        if not self.tag_detected:
            rospy.loginfo("Tag found! Switching to look-at behavior.")
        self.tag_detected = True

        x = detections[0].transform.translation.x
        z = detections[0].transform.translation.z  # distance (for logging)
        rospy.loginfo("Tag x=%.3f  z=%.3f", x, z)

        # Proportional control: omega is proportional to horizontal offset (x)
        # x > 0 means tag is to the LEFT  → turn left  (positive omega)
        # x < 0 means tag is to the RIGHT → turn right (negative omega)

        KP = 3.0          # Proportional gain — increase if response is sluggish
        MAX_OMEGA = 3.0   # Cap to avoid spinning too fast (rad/s)
        MIN_OMEGA = 0.5   # Minimum to overcome friction (rad/s)
        DEADZONE  = 0.05  # If tag is this centered, don't turn (metres)

        if abs(x) < DEADZONE:
            cmd_msg.omega = 0.0  # Tag is centered, hold still
        else:
            omega = KP * x
            # Clamp to max
            omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))
            # Apply minimum to overcome motor friction
            if 0 < omega < MIN_OMEGA:
                omega = MIN_OMEGA
            elif -MIN_OMEGA < omega < 0:
                omega = -MIN_OMEGA
            cmd_msg.omega = omega

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass