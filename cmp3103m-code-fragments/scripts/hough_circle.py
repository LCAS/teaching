import cv2
import numpy as np

import cv2.cv as cv  

cv2.namedWindow("detected circles", 1)
cv2.startWindowThread()

cimg = cv2.imread('../blofeld.jpg')
img = cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY)
img = cv2.medianBlur(img,5)

circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,4,20,
                            param1=230,param2=230,minRadius=0,maxRadius=0)
if circles is not None:
    print len(circles)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
