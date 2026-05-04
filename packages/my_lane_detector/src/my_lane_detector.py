#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### REMEMBER TO CHANGE THE TOPIC NAME! #####        
        self.image_sub = rospy.Subscriber('/akandb/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        #############################################

        rospy.init_node("my_lane_detector")

    def image_callback(self,msg):
        rospy.loginfo("image_callback")


        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        #### YOUR CODE GOES HERE ####
        # crop rioad region
        h, w, _ = img.shape
        cropped = img[int(h*0.5):h, :]

        # hsv conversion
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # white mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        # yellow mask
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # canny edge detection
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        edges = cv2.Canny(gray, 50, 150)

        # hough transform (white)
        white_edges = cv2.Canny(white_mask, 50, 150)
        white_lines = cv2.HoughLinesP(
            white_edges,
            1,
            np.pi/180,
            threshold=50,
            minLineLength=20,
            maxLineGap=50
        )

        # hough transform (yellow)
        yellow_edges = cv2.Canny(yellow_mask, 50, 150)
        yellow_lines = cv2.HoughLinesP(
            yellow_edges,
            1,
            np.pi/180,
            threshold=50,
            minLineLength=20,
            maxLineGap=50
        )

        # draw lines
        output = cropped.copy()

        if white_lines is not None:
            for line in white_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1,y1), (x2,y2), (255,255,255), 2)

        if yellow_lines is not None:
            for line in yellow_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1,y1), (x2,y2), (0,255,255), 2)

        # display
        cv2.imshow("cropped", cropped)
        cv2.imshow("white_mask", white_mask)
        cv2.imshow("yellow_mask", yellow_mask)
        cv2.imshow("edges", edges)
        cv2.imshow("hough_transformation", output)

        cv2.waitKey(1)

    def run(self):
    	rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    
    
