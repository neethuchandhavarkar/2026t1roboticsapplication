#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped


class ClosedLoopController:
    def __init__(self):
        rospy.init_node('closed_loop_controller')

        self.pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd',
                                   Twist2DStamped, queue_size=1)

        rospy.Subscriber('/mybota002409/fsm_node/mode',
                         FSMState, self.fsm_callback)

        rospy.Subscriber('/mybota002409/left_wheel_encoder_node/tick',
                         WheelEncoderStamped, self.encoder_callback)
        
        rospy.Subscriber('/mybota002409/fsm_node/mode',
                         WheelEncoderStamped, self.encoder_callback)

    
        # "STRAIGHT", "ROTATE", "SQUARE"
        self.MODE = "SQUARE"

        # Encoder tracking
        self.start_ticks = 0
        self.current_ticks = 0

        # Motion state
        self.state = "IDLE"
        self.target_ticks = 0

        # CALIBRATE 
        self.TICKS_PER_METER = 650
        self.TICKS_PER_90_DEG = 300

        self.cmd = Twist2DStamped()

        # For square
        self.step = 0

  
    def fsm_callback(self, msg):
        if self.state == False:
            rospy.loginfo(f"Starting mode: {self.MODE}")
            rospy.sleep(1)

            if self.MODE == "STRAIGHT":
                self.run_straight_test()

            elif self.MODE == "ROTATE":
                self.run_rotation_test()

            elif self.MODE == "SQUARE":
                self.start_square()

        else:
            self.stop_robot()

   
    def encoder_callback(self, msg):
        self.current_ticks = msg.data

        if self.state == "MOVING":
            moved = abs(self.current_ticks - self.start_ticks)

            if moved >= self.target_ticks:
                rospy.loginfo("Target reached")
                self.stop_robot()
                self.next_action()


    # Motion functions
    
    def move_straight(self, distance, speed):
        rospy.loginfo(f"Move {distance}m at speed {speed}")

        self.start_ticks = self.current_ticks
        self.target_ticks = abs(distance) * self.TICKS_PER_METER

        self.cmd.v = speed
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)

        self.state = "MOVING"

    def rotate_in_place(self, angle_deg, omega):
        rospy.loginfo(f"Rotate {angle_deg}° at omega {omega}")

        self.start_ticks = self.current_ticks
        self.target_ticks = (abs(angle_deg) / 90.0) * self.TICKS_PER_90_DEG

        self.cmd.v = 0.0
        self.cmd.omega = omega
        self.pub.publish(self.cmd)

        self.state = "MOVING"

    def stop_robot(self):
        self.cmd.v = 0.0
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)
        self.state = "STOPPED"

    
    # TEST MODES
    

    #  Straight test (2 speeds + forward/backward)
    def run_straight_test(self):
        self.test_sequence = [
            ("straight", 1.0, 0.2),   # slow forward
            ("straight", 1.0, 0.4),   # fast forward
            ("straight", -1.0, -0.2),  # slow backward
            ("straight", -1.0, -0.4) # fast backward
        ]
        self.test_step = 0
        self.run_test_step()

    # Rotation test (2 speeds + both directions)
    def run_rotation_test(self):
        self.test_sequence = [
            ("rotate", 90, 3.0),    # CCW slow
            ("rotate", 90, 5.0),    # CCW fast
            ("rotate", 90, -3.0),   # ACW slow
            ("rotate", 90, -5.0)    # ACW fast
            
        ]
        self.test_step = 0
        self.run_test_step()

    def run_test_step(self):
        if self.test_step >= len(self.test_sequence):
            rospy.loginfo("Test complete")
            self.stop_robot()
            return

        action, value, speed = self.test_sequence[self.test_step]

        if action == "straight":
            self.move_straight(value, speed)
        else:
            self.rotate_in_place(value, speed)

    # Square
    
    def start_square(self):
        rospy.loginfo("Starting square")
        self.step = 0
        self.do_square_step()

    def do_square_step(self):
        if self.step >= 8:
            rospy.loginfo("Square complete")
            self.stop_robot()
            return

        if self.step % 2 == 0:
            self.move_straight(1.0, 0.3)
        else:
            self.rotate_in_place(90, 4.0)

    def next_action(self):
        if self.MODE == "SQUARE":
            self.step += 1
            self.do_square_step()
        else:
            self.test_step += 1
            self.run_test_step()

    # ======================
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = ClosedLoopController()
    node.run()