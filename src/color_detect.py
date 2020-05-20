import cv2
import numpy as np
import imutils



class Color_detect:
    def __init__(self, vs=4):#, x_cor, y_cor):
        # self.x_cor = x_cor
        # self.y_cor = y_cor

        self.cap = cv2.VideoCapture(vs)  # begins video capture from a camera plugged into the usb, in this case vs = 4

    def return_centroids_frame(self):
        _, frame = self.cap.read()  # stores what is being captured from the camera into frame

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # converts it into hsv


        # Green Color

        # 75-86, 150-255, 5-255 for green tape

        #color specification ranges, can have two different ranges
        low_green = np.array([0, 150, 5])  #this range is currently for the red tape
        high_green = np.array([5, 210, 255])

        low = np.array([170,150,5]) # also red tape
        high = np.array([255,210,255])


        green_mask =  cv2.inRange(hsv_frame, low_green, high_green) | cv2.inRange(hsv_frame,low,high) # looks at hsv frame and determines which pixels are within either of two specified color ranges
        green = cv2.bitwise_and(frame, frame, mask=green_mask) # blacks out everything that is not within that color range



        ##  Noise Handling ##

        # simple average
        kernel = np.ones((15,15), np.float32)/225
        smoothed = cv2.filter2D(green,-1,kernel)

        # gaussian blur
        blur = cv2.GaussianBlur(green, (15,15), 0)

        # median blur
        median = cv2.medianBlur(green,15)

        ## FINDING CONTOURS ##

        cnts = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        cx = 0
        cy = 0


        for c in cnts:

            # to find area of the shape
            area = cv2.contourArea(c)

            #draws contours on the image
            cv2.drawContours(median,[c],-1,(0,255,0),3)

            #finds the contour with the biggest area
            d = max(cnts, key = cv2.contourArea)

            #finds teh centroid of that area
            M = cv2.moments(d)

            cx = int(M["m10"]/(M["m00"]+1e-7))
            cy = int(M["m01"]/(M["m00"]+1e-7))

            #displays a dot on the image where the centroid is
            cv2.circle(median,(cx,cy),7,(255,255,255),-1)
            cv2.putText(median, "Center", (cx-20, cy -20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        #cv2.imshow("Frame", frame)
        ##cv2.imshow("Green", green)
        #cv2.imshow("Median", median)

        key = cv2.waitKey(1)





        return cx, cy, median, hsv_frame


if __name__ == '__main__':
    cd = Color_detect()
    while True:
        cx, cy, image = cd.return_centroids_frame()
        print(cx)
        print(cy)
        cv2.imshow('green_mask', image)
