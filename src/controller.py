#!/usr/bin/env python3
import rospy
import imutils
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import Empty


xc = 320  # the pixel location of the center of the screen, this will probably change
yc = 240

ex_old = 0
ex_old2 = 0

ey_old = 0
ey_old2 = 0

class controller():
    def __init__(self):
        self.centroid_sub = rospy.Subscriber('centroid_topic',Point, self.centroid_receiver, queue_size=1000)
        self.servo_pub = rospy.Publisher('servo_topic',Point, queue_size =1000)
        #self.control_pub = rospy.Publisher('control_command',Point,queue_size=1000)
        while not rospy.is_shutdown():
            print("while not")

            rospy.spin()



    def centroid_receiver(self, data): #this looks like the callback function. is this what is called everytime the node grabs something from the topic???
        x = data.x
        y = data.y

        ex = x - xc  #error, difference between the measured x value and the desired x value, which should be the center of the screen
        ey = y - yc

        #ex_smooth = (ex + ex_old + ex_old2)/3
        #ey_smooth = (ey + ey_old + ey_old2)/3

        #ex_old2 = ex_old
        #ex_old = ex

        #ey_old2 = ey_old
        #ey_old = ey

        servo_msg = Point()

        servo_msg.x = ex
        servo_msg.y = ey
        self.servo_pub.publish(servo_msg)

        #this data is pretty noisy, so I will want to perform some kind of regression on it to smooth it out
        #could just take an average of the previous ten coordinates together






if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    Controller = controller()
