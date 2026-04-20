#!/usr/bin/env python3
import rospy
import time
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range


class ClosedLoopController:
    def __init__(self):
        rospy.init_node('closed_loop_controller')

        self.pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        rospy.Subscriber('/mybota002409/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)
        rospy.Subscriber('/mybota002409/fsm_node/mode', FSMState, self.fsm_callback)
        rospy.Subscriber('/mybota002409/front_center_tof_driver_node/range', Range, self.tof_callback)

        self.tof_distance = float('inf')
        self.OBSTACLE_THRESHOLD = 0.25      # meters
        self.paused_for_obstacle = False

        self.MODE = "TEST"
        self.phase = 0

        # Prevent multiple starts
        self.started = False
        self.action_done = False

        # Encoder tracking
        self.start_ticks = 0
        self.current_ticks = 0

        # Motion state
        self.state = "IDLE"
        self.target_ticks = 0

        # Calibration 
        self.TICKS_PER_METER = 220
        self.TICKS_PER_90_DEG = 35

        self.cmd = Twist2DStamped()
        self.prev_cmd = 0

        # For square
        self.test_step = 0
        self.test_sequence = []

    
    # FSM CALLBACK
    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING" and not self.started:
            self.started = True
            self.phase = 0

            self.run_square_test()
            
    # ENCODER CALLBACK
    def encoder_callback(self, msg):
        self.current_ticks = msg.data
        
        # OBSTACLE DETECTED
        if self.tof_distance < self.OBSTACLE_THRESHOLD:
            if not self.paused_for_obstacle:
                rospy.loginfo("Obstacle detected! Stopping")
                self.prev_cmd = self.cmd.v
                self.prev_ticks = self.current_ticks
                self.stop_robot()
                self.paused_for_obstacle = True
            return

        # OBSTACLE CLEARED --> RESUME
        if self.paused_for_obstacle and self.tof_distance >= self.OBSTACLE_THRESHOLD:
            rospy.loginfo("Obstacle cleared! Resuming")
            self.paused_for_obstacle = False
            self.current_action()

        # NORMAL CLOSED-LOOP LOGIC
        if self.state == "MOVING" and not self.paused_for_obstacle:
            moved = abs(self.current_ticks - self.start_ticks)
            
            if moved >= self.target_ticks and not self.action_done:
                self.action_done = True
                rospy.loginfo("Target reached")
                self.stop_robot()
                self.next_action()

    # Tof callback
    def tof_callback(self, msg):
        self.tof_distance = msg.range

    # MOTION FUNCTIONS
    def move_straight(self, distance, speed):
        self.action_done = False
        rospy.loginfo(f"Move {distance}m at speed {speed}")

        self.start_ticks = self.current_ticks
        self.target_ticks = abs(distance) * self.TICKS_PER_METER

        self.cmd.v = speed
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)

        self.state = "MOVING"

    def rotate_in_place(self, angle_deg, omega):
        self.action_done = False

        rospy.loginfo(f"Rotate {angle_deg}° at omega {omega}")

        self.start_ticks = self.current_ticks
        self.target_ticks = (abs(angle_deg) / 90.0) * self.TICKS_PER_90_DEG

        self.cmd.v = 0.0
        self.cmd.omega = abs(omega) if angle_deg > 0 else -abs(omega)
        self.pub.publish(self.cmd)

        self.state = "MOVING"

    def stop_robot(self):
        self.cmd.v = 0.0
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)
        self.state = "STOPPED"


    # SQUARE
    def run_square_test(self):
        self.test_sequence = [
            ("straight", 1.0, 0.2),
            ("rotate", 90, 3.0),
            ("straight", 1.0, 0.2),
            ("rotate", 90, 3.0),
            ("straight", 1.0, 0.2),
            ("rotate", 90, 3.0),
            ("straight", 1.0, 0.2),
            ("rotate", 90, 3.0)
        ]
        self.test_step = 0

        self.state = "IDLE"
        rospy.sleep(0.5)

        self.run_test_step()

    def run_test_step(self):
        if self.test_step >= len(self.test_sequence):
            rospy.loginfo("Test complete")
            self.stop_robot()
            return 

        action, value, speed = self.test_sequence[self.test_step]

        if action == "straight":
            self.move_straight(value, speed)
            rospy.sleep(0.5)
        else:
            self.rotate_in_place(value, speed)
            rospy.sleep(0.5)

    def current_action(self):
        if self.test_step < len(self.test_sequence):
            self.run_test_step()
            return

        else:
            rospy.loginfo(f"Test Complete")
            self.stop_robot()
            self.started = False


    def next_action(self):
        self.test_step += 1

        if self.test_step < len(self.test_sequence):
            self.run_test_step()
            return

        else:
            rospy.loginfo(f"Test Complete")
            self.stop_robot()
            self.started = False


    # ======================
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = ClosedLoopController()
    node.run()