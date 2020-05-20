#!/usr/bin/env python3
import rospy
import imutils
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from matplotlib import pyplot as plt
import numpy as np

# the pixel coordinates of the center of the frame
xc = 320
yc = 240

# initializing variables
ex_old = 0
ex_old2 = 0

ey_old = 0
ey_old2 = 0

time = 0

class controller():
    def __init__(self):
        #define this node as a subscriber to the 'centroid_topic', which is getting Point messages from color_detect_ros.py
        self.centroid_sub = rospy.Subscriber('centroid_topic',Point, self.centroid_receiver, queue_size=1000)

        #define this node as a publisher to the 'servo_topic', publishing Point messages
        self.servo_pub = rospy.Publisher('servo_topic',Point, queue_size =1000)

        while not rospy.is_shutdown():
            print("while not")

            #plt.plot(1,2, 'rp')
            #plt.show()

            rospy.spin()



    #defining the subscriber callback function
    def centroid_receiver(self, data):
        global ex_old
        global ex_old2
        global ey_old
        global ey_old2
        global time

        time = time + 1

        # store the values pulled in from the received Point message
        x = data.x
        y = data.y

        #error, difference between the measured x value and the desired x value, which should be the center of the screen
        ex = x - xc
        ey = y - yc

        # takes a moving average of the previous three coordinates
        ex_smooth = round((ex + ex_old + ex_old2)/3)
        ey_smooth = round((ey + ey_old + ey_old2)/3)

        #reassigns these values
        ex_old2 = ex_old
        ex_old = ex

        ey_old2 = ey_old
        ey_old = ey

        #define the outgoing message to be a Point message
        servo_msg = Point()

        # stores the smoothed data point into the outgoing message
        servo_msg.x = ex_smooth
        servo_msg.y = ey_smooth

        # publishes the outgoing message
        self.servo_pub.publish(servo_msg)







if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    Controller = controller()
