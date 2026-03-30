#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.pose = None

        self.goal_distance = 0
        self.start_x = 0
        self.start_y = 0
        
        self.goal_position = None

        self.goal_angle = 0
        self.start_angle = 0
        
        self.dist_goal_active = False
        self.angle_goal_active = False
        self.pos_goal_active = False
        
        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64,self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64,self.goal_angle_callback)
        rospy.Subscriber("/goal_position", Point,self.goal_position_callback)
        rospy.Subscriber("/goal_distance", Float64,self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # callbacks
    def pose_callback(self,msg):
        self.pose = msg

    def distance_callback(self,msg):
        self.last_distance = msg.data


    # angle
    def goal_angle_callback(self,msg):
        if self.pose is None:
            return

        self.goal_angle = msg.data
        self.start_angle = self.pose.theta

        self.angle_goal_active = True
        self.dist_goal_active = False
        self.pos_goal_active = False

        rospy.loginfo(f"Angle goal: {self.goal_angle}")
    
    # distance
    def goal_distance_callback(self,msg):
        ########## YOUR CODE GOES HERE ##########
        # Set goal_distance, dist_goal_active and forward_movement variables here
        if self.pose is None:
            return

        self.goal_distance = msg.data
        
        self.start_x = self.pose.x
        self.start_y = self.pose.y
        
        self.dist_goal_active = True
        self.angle_goal_active = False
        self.pos_goal_active = False

        rospy.loginfo(f"Distance goal: {self.goal_distance}")

        ###########################################    
        
    # position
    def goal_position_callback(self,msg):
        self.goal_position = msg

        self.pos_goal_active = True
        self.angle_goal_active = False
        self.dist_goal_active = False

        rospy.loginfo(f"Position goal: {msg.x}, {msg.y}")


    def angle_diff(self, a, b): 
        diff = a - b
        return math.atan2(math.sin(diff), math.cos(diff))


    def timer_callback(self,msg):
        ########## YOUR CODE GOES HERE ##########
        # If a goal is active, first check if the goal is reached (it's OK if the goal is not perfectly reached)
        # Then publish a cmd_vel message

        if self.pose is None:
            return

        vel = Twist()

        # distance
        if self.dist_goal_active:
            if abs(self.goal_distance) < 0.001:
                self.dist_goal_active = False
            
            else:

                dx = self.pose.x - self.start_x
                dy = self.pose.y - self.start_y

                traveled = math.sqrt(dx*dx + dy*dy)
                    
                if traveled >= abs(self.goal_distance):
                    rospy.loginfo("Distance reached")
                    self.dist_goal_active = False
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0

                else:
                    vel.linear.x = 1.0 if self.goal_distance > 0 else -1.0

        # angle
        elif self.angle_goal_active:
            if abs(self.goal_angle) < 0.001:
                self.angle_goal_active = False
                return
            
            current = self.pose.theta

            diff = self.angle_diff(current, self.start_angle)

            if abs(diff) >= abs(self.goal_angle):
                rospy.loginfo("Angle reached")
                self.angle_goal_active = False

            else:
                if self.goal_angle > 0:
                    vel.angular.z = 1.0
                else:
                    vel.angular.z = -1.0

        # position
        elif self.pos_goal_active:
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y
            
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < 0.1:
                rospy.loginfo("Position reached")
                self.pos_goal_active = False

            else:
                target_angle = math.atan2(dy, dx)

                angle_error = self.angle_diff(target_angle, self.pose.theta)

                if abs(angle_error) > 0.1:
                    vel.angular.z = (1.0 if angle_error > 0 else -1.0)

                else:
                    vel.linear.x = 1.0

        self.velocity_publisher.publish(vel)

        ###########################################

if __name__ == '__main__': 

    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
        
