#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/mybota002409/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/mybota002409/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        self.move_robot(msg.detections) == 0
        return

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

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):

        #### YOUR CODE GOES HERE ####

        if len(detections) == 0:
            return

        # Read the first detected tag's ID
        tag_id = detections[0].tag_id

        # --- STOP SIGN ---
        if tag_id in [21, 22]:
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(3.0)                  # Wait 3 seconds at stop sign
            self.set_state("LANE_FOLLOWING")

        # --- RIGHT TURN ---
        elif tag_id in [56, 57]:
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(0.5)                  # Brief pause before turning

            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.3                   # Move forward
            cmd_msg.omega = -2.5              # Negative = turn right
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(1.8)                  # Tune this duration for your robot

            self.stop_robot()
            rospy.sleep(0.3)
            self.set_state("LANE_FOLLOWING")

        # --- LEFT TURN ---
        elif tag_id in [58, 59]:
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(0.5)

            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.3                   # Move forward
            cmd_msg.omega = 2.5               # Positive = turn left
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(2.2)                  # Left turns typically need more time

            self.stop_robot()
            rospy.sleep(0.3)
            self.set_state("LANE_FOLLOWING")

        # --- OBSTACLE ---
        elif tag_id in [163, 164]:
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(1.0)                  # Wait to see if obstacle moves

            # Swerve left to avoid
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.3
            cmd_msg.omega = 2.0               # Steer left around obstacle
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(1.0)

            # Straighten up and pass
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(1.0)

            self.stop_robot()
            self.set_state("LANE_FOLLOWING")
      
        

        #############################

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass