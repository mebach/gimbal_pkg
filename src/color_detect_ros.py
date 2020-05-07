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
        while not rospy.is_shutdown():
            cx, cy, image = self.cd.return_centroids_frame()
            centroid_msg = Point()
            centroid_msg.x = cx
            centroid_msg.y = cy
            cv2.imshow('green_mask',image)
            self.color_pub.publish(centroid_msg)



if __name__ == '__main__':
    rospy.init_node('color_detect_ros', anonymous=True)
    Color_detect_ros = color_detect_ros()
