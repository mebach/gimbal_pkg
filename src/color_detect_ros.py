#!/usr/bin/env python3
import rospy
import imutils
import cv2
from geometry_msgs.msg import Point
from color_detect import Color_detect


class color_detect_ros():
    def __init__(self):
        self.color_pub = rospy.Publisher('centroid_topic', Point, queue_size=1000)
        self.cd = Color_detect()
        cv2.namedWindow('hsv_frame')

        while not rospy.is_shutdown():
            cx, cy, image, hsv_frame = self.cd.return_centroids_frame()
            centroid_msg = Point()
            centroid_msg.x = cx
            centroid_msg.y = cy

            cv2.setMouseCallback('hsv_frame',self.mouseRGB,hsv_frame)


            cv2.imshow('green_mask',image)
            cv2.imshow('hsv_frame',hsv_frame)

            self.color_pub.publish(centroid_msg)

    def mouseRGB(self,event,x,y,flags,frame):
        if event == cv2.EVENT_LBUTTONDOWN:
            colorsB = frame[y,x,0]
            colorsG = frame[y,x,1]
            colorsR = frame[y,x,2]
            colors = frame[y,x]
            print("HSV format: ", colors)



if __name__ == '__main__':
    rospy.init_node('color_detect_ros', anonymous=True)
    Color_detect_ros = color_detect_ros()
