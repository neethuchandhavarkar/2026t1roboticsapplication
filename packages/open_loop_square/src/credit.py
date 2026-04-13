#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class ClosedLoopSquare:
    def __init__(self):
        rospy.init_node('closed_loop_square')

        self.pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        rospy.Subscriber('/mybota002409/fsm_node/mode', FSMState, self.fsm_callback)

        rospy.Subscriber('/mybota002409/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)

        # Motion state
        self.state = "IDLE"

        # Encoder tracking
        self.start_ticks = 0
        self.current_ticks = 0

        # Goals
        self.target_ticks = 0

        # Calibration change it according to the robot
        self.TICKS_PER_METER = 355
        self.TICKS_PER_90_DEG = 20

        self.cmd = Twist2DStamped()

    # FSM
    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.loginfo("Starting square")
            rospy.sleep(1)
            self.start_square()

    # Encoder callback
    def encoder_callback(self, msg):
        self.current_ticks = msg.data

        if self.state == "MOVING":
            moved = abs(self.current_ticks - self.start_ticks)

            if moved >= self.target_ticks:
                rospy.loginfo("Reached target")
                self.stop_robot()
                self.next_action()

    # Motion 
    def move_straight(self, distance):
        self.start_ticks = self.current_ticks
        self.target_ticks = distance * self.TICKS_PER_METER

        self.cmd.v = 0.3
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)

        self.state = "MOVING"

    def rotate_in_place(self, angle_deg):
        self.start_ticks = self.current_ticks
        self.target_ticks = (angle_deg / 90.0) * self.TICKS_PER_90_DEG

        self.cmd.v = 0.0
        self.cmd.omega = 4.0
        self.pub.publish(self.cmd)

        self.state = "MOVING"

    def stop_robot(self):
        self.cmd.v = 0.0
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)
        self.state = "STOPPED"

    # Square logic
    def start_square(self):
        self.step = 0
        self.do_step()

    def do_step(self):
        if self.step >= 8:
            rospy.loginfo("Square complete")
            self.stop_robot()
            return

        if self.step % 2 == 0:
            rospy.loginfo("Forward")
            self.move_straight(1.0)
        else:
            rospy.loginfo("Turn")
            self.rotate_in_place(90)

    def next_action(self):
        self.step += 1
        self.do_step()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = ClosedLoopSquare()
    node.run()