#!/usr/bin/env python3
import rospy
import imutils
import cv2
from geometry_msgs.msg import Point
from color_detect import Color_detect


class color_detect_ros():
    def __init__(self):
        # defines this node as publisher, publishing Point messages to the topic 'centroid_topic'
        self.color_pub = rospy.Publisher('centroid_topic', Point, queue_size=1000)

        # instantiates self as Color_detect, the class that is defined in the color_detect.py file
        self.cd = Color_detect()
        cv2.namedWindow('hsv_frame')

        while not rospy.is_shutdown():

            #pulls the coordinates out of the function
            cx, cy, image, hsv_frame = self.cd.return_centroids_frame()

            #stores those values into the message that will be published
            centroid_msg = Point()
            centroid_msg.x = cx
            centroid_msg.y = cy

            # when the mouse is pressed, call mouseRGB function
            cv2.setMouseCallback('hsv_frame',self.mouseRGB,hsv_frame)

            # displays the actual iamges
            cv2.imshow('green_mask',image)
            cv2.imshow('hsv_frame',hsv_frame)

            # publishes the centroid coordinates
            self.color_pub.publish(centroid_msg)

    def mouseRGB(self,event,x,y,flags,frame):

        # when the left mouse button is pressed...
        if event == cv2.EVENT_LBUTTONDOWN:
            colorsB = frame[y,x,0]
            colorsG = frame[y,x,1]
            colorsR = frame[y,x,2]
            colors = frame[y,x]

            #print the hsv values of the pixel that was clicked on
            print("HSV format: ", colors)



if __name__ == '__main__':
    rospy.init_node('color_detect_ros', anonymous=True)
    Color_detect_ros = color_detect_ros()
