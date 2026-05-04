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

class LaneAnalysis:
    def __init__(self):
        rospy.init_node("lane_analysis")
        
        self.cv_bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/mybota002409/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)

        self.frame_count = 0

        # canny threshold pairs
        self.canny_settings = [
            (50, 150),
            (100, 200),
            (150, 250)
        ]

        # hough parameters variations (minLineLength)
        self.hough_lengths = [20, 50, 100]

    def callback(self,msg):
        rospy.loginfo("callback")

        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # crop rioad region
        h, w, _ = img.shape
        cropped = img[int(h*0.5):h, :]

        # Canny experiment
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        for i, (low, high) in enumurate(self.canny_settings):
            edges = cv2.Canny(gray, low, high)

            cv2.imshow(f"Canny_{low}_{high}", edges)

            if self.frame_count % 450 == 0:
                cv2.imwrite(f"canny_{low}_{high}_{frame_count}.png", edges)

        # hough experiment
        edges = cv2.Canny(gray, 50, 150)
        
        for length in self.hough_lengths:
            lines = cv2.HoughLinesP(
                edges,
                1,
                np.pi/180,
                threshold=50,
                minLineLength=length,
                maxLineGap=50
            )

            output = cropped.copy()

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(output, (x1, y1), (x2, y2), (0,255,0), 2)
            
            cv2.imshow("Hough_len_{length}", output)

            if self.frame_count % 450 = 0:
                cv2.imwrite(f"hough_len{length}_{frame_count}.png", output)

        # HSV vs RGB
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        hsv_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_yellow_rgb = np.array([0, 200, 200])
        upper_yellow_rgb = np.array([100, 255, 255])
        rgb_mask = cv2.inRange(cropped, lower_yellow_rgb, upper_yellow_rgb)

        cv2.imshow("HSV_yellow", hsv_mask)
        cv2.imshow("RGB_yellow", rgb_mask)

        if self.frame_count % 450 == 0:
            cv2.imwrite(f"hsv_yellow.png_{frame_count}", hsv_mask)
            cv2.imwrite(f"rgb_yellow.png_{frame_count}", rgb_mask)

        # lighting
        darker = cv2.convertScaleAbs(cropped, alpha=0.5, beta=0)
        brighter = cv2.convertScaleAbs(cropped, alpha=1.5, beta=50)

        edges_dark = cv2.Canny(cv2.cvtColor(darker, cv2.COLOR_BGR2HSV), 50, 150)
        edges_bright = cv2.Canny(cv2.cvtColor(brighter, cv2.COLOR_BGR2HSV), 50, 150)

        cv2.imshow("Dark", edges_dark)
        cv2.imshow("Bright", edges_bright)

        if self.frame_count % 450 == 0:
            cv2.imwrite(f"dark_edges_{frame_count}.png", hsv_mask)
            cv2.imwrite(f"bright_edges_{frame_count}.png", rgb_mask)

        self.frame_count += 1
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        node = LaneAnalysis()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    
