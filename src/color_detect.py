import cv2
import numpy as np
import imutils



class Color_detect:
    def __init__(self, vs=4):#, x_cor, y_cor):
        # self.x_cor = x_cor
        # self.y_cor = y_cor

        self.cap = cv2.VideoCapture(vs)

    def return_centroids_frame(self):
        _, frame = self.cap.read()

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        # Green Color

        low_green = np.array([75, 125, 5]) # THESE RANGES ARE PRETTY GOOD FOR THE GREEN TAPE, IT MIGHT NOT BE GOOD FOR THE VAPORLITE
        high_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv_frame, low_green, high_green)
        green = cv2.bitwise_and(frame, frame, mask=green_mask)

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

            #find contours
            cv2.drawContours(median,[c],-1,(0,255,0),3)

            d = max(cnts, key = cv2.contourArea)


            M = cv2.moments(d)

            #print (M["m00"])

            cx = int(M["m10"]/(M["m00"]+1e-7))
            cy = int(M["m01"]/(M["m00"]+1e-7))

            # centroid = color_detect(cx,cy)
            #
            # print(centroid.x_cor)
            # print(centroid.y_cor)


            cv2.circle(median,(cx,cy),7,(255,255,255),-1)
            cv2.putText(median, "Center", (cx-20, cy -20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        #cv2.imshow("Frame", frame)
        ##cv2.imshow("Green", green)
        #cv2.imshow("Median", median)

        key = cv2.waitKey(1)



        return cx, cy, median


if __name__ == '__main__':
    cd = Color_detect()
    while True:
        cx, cy, image = cd.return_centroids_frame()
        print(cx)
        print(cy)
        cv2.imshow('green_mask', image)
