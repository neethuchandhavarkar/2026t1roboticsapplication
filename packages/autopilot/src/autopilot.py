#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Range
class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        # Setting and variables
        self.bot = "mybota002409"

        self.mode = "LANE_FOLLOWING"

        self.current_distance = 999.0

        self.left_ticks = 0
        self.right_ticks = 0

        self.moving = False
        self.target_ticks = 0
        self.start_left_ticks = 0

        # calibration
        self.TICKS_FORWARD_SMALL = 80
        self.TICKS_FORWARD_MEDIUM = 140
        self.TICKS_90_DEG = 55

        # tag IDs
        self.STOP_SIGN_ID = 162
        self.LEFT_TURN_ID = 153
        self.RIGHT_TURN_ID = 58

        # obstacle threshold
        self.OBSTACLE_DISTANCE = 0.25
        
        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher(f'/{self.bot}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher(f'/{self.bot}/fsm_node/mode', FSMState, queue_size=1)

        # subs
        rospy.Subscriber(
            f'/{self.bot}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        rospy.Subscriber(
            f'/{self.bot}/front_center_tof_driver_node/range',
            Range,
            self.tof_callback,
            queue_size=1
        )

        rospy.Subscriber(
            f'/{self.bot}/left_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.left_encoder_callback
        )

        rospy.Subscriber(
            f'/{self.bot}/right_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.right_encoder_callback
        )

        rospy.spin() # Spin forever but listen to message callbacks

    # =============== Encoders ============
    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    # =============== TOF ============
    def tof_callback(self, msg):
        self.current_distance = msg.range

        # obstacle stop
        if self.current_distance < self.OBSTACLE_DISTANCE:
            if self.mode == "LANE_FOLLOWING":
                rospy.loginfo("Obstacle detected")
                self.mode = "OBSTACLE_STOP"
                self.set_state("NORMAL_JOYSTICK_CONTROL")
                self.stop_robot()

        else:
            # obstacle removed
            if self.mode == "OBSTACLE_STOP":
                rospy.loginfo("Obstacle cleared")
                self.mode = "LANE_FOLLOWING"
                self.set_state("LANE_FOLLOWING")


    # =============== Apriltag ============
    def tag_callback(self, msg):
        detections = msg.detections

        # STOP SIGN WAIT
        if self.mode == "STOP_SIGN":

            if len(detections) == 0:

                rospy.loginfo("Stop sign removed")
                self.mode = "LANE_FOLLOWING"
                self.set_state("LANE_FOLLOWING")

            return

        # ignore tags if busy
        if self.mode != "LANE_FOLLOWING":
            return

        if len(detections) == 0:
            return

        tag_id = detections[0].tag_id

        rospy.loginfo(f"Detected tag: {tag_id}")

        # STOP SIGN
        if tag_id == self.STOP_SIGN_ID:

            rospy.loginfo("STOP SIGN")
            self.mode = "STOP_SIGN"
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()

        # LEFT TURN
        elif tag_id == self.LEFT_TURN_ID:

            rospy.loginfo("LEFT TURN")
            self.mode = "INTERSECTION"
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            rospy.sleep(1)
            self.handle_left_turn()
            self.mode = "LANE_FOLLOWING"
            self.set_state("LANE_FOLLOWING")

        # RIGHT TURN
        elif tag_id == self.RIGHT_TURN_ID:

            rospy.loginfo("RIGHT TURN")
            self.mode = "INTERSECTION"
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            rospy.sleep(1)
            self.handle_right_turn()
            self.mode = "LANE_FOLLOWING"
            self.set_state("LANE_FOLLOWING")

    # =============== LEFT INTERSECTION ============
    def handle_left_turn(self):
        # move to red line
        self.drive_ticks(self.TICKS_FORWARD_SMALL)

        # rotate left
        self.rotate_ticks(self.TICKS_90_DEG)

        # move into next lane
        self.drive_ticks(self.TICKS_FORWARD_MEDIUM)


    # =============== RIGHT INTERSECTION ============
    def handle_right_turn(self):
        self.drive_ticks(self.TICKS_FORWARD_SMALL)
        self.rotate_ticks(-self.TICKS_90_DEG)
        self.drive_ticks(self.TICKS_FORWARD_MEDIUM)

    # =============== OVERTAKE ============
    def overtake(self):

        rospy.loginfo("Starting overtake")

        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # left
        self.rotate_ticks(25)
        self.drive_ticks(80)
        self.rotate_ticks(-25)

        # pass obstacle
        self.drive_ticks(120)

        # return
        self.rotate_ticks(-25)
        self.drive_ticks(80)
        self.rotate_ticks(25)

        rospy.loginfo("Overtake complete")

        self.set_state("LANE_FOLLOWING")

    # =============== CLOSED LOOP MOVEMENT ============
    def drive_ticks(self, ticks):

        start = self.left_ticks

        cmd = Twist2DStamped()
        cmd.v = 0.25
        cmd.omega = 0.0

        rate = rospy.Rate(30)

        while abs(self.left_ticks - start) < ticks and not rospy.is_shutdown():
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)

    # =============== ROTATION MOVEMENT ============
    def rotate_ticks(self, ticks):

        start = self.left_ticks

        cmd = Twist2DStamped()
        cmd.v = 0.0

        if ticks > 0:
            cmd.omega = 3.0
        else:
            cmd.omega = -3.0

        rate = rospy.Rate(30)

        while abs(self.left_ticks - start) < abs(ticks) and not rospy.is_shutdown():
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)

    # =============== STOP ROBOT & SHUTDOWN ============ 
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

    # =============== FSM MODE ============ 
    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)


if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass