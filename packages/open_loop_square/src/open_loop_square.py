#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
 
class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybota002409/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.move_robot()
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
    	rospy.spin() # keeps node from exiting until node has shutdown

    # Drive straight
    def drive_straight(self, duration, speed=0.3):
        rospy.loginfo("Driving straight")

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

        rospy.sleep(duration)
        self.stop_robot()

    # turn 90 degrees
    def turn_90(self, duration, angular_speed=4.0):
        rospy.loginfo("Turning 90 degrees")

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = angular_speed
        self.pub.publish(self.cmd_msg)

        rospy.sleep(duration)
        self.stop_robot()

    # Make square
    def make_square(self):
        rospy.loginfo("Starting square path")

        for i in range(4):
            rospy.loginfo(f"Side {i+1}")
            self.drive_straight(duration=2.0)
            self.turn_90(duration=.8)

    # Robot drives in a square and then stops
    def move_robot(self):

        #YOUR CODE GOES HERE#
        self.make_square()                
        self.stop_robot()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass