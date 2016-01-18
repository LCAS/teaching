from cv2 import namedWindow, imread, imshow
from cv2 import waitKey, destroyAllWindows, startWindowThread
from cv2 import blur, Canny

import numpy

namedWindow("original")
namedWindow("blur")
namedWindow("canny")

startWindowThread()

img = imread('../blofeld.jpg')
imshow("original", img)

img2 = blur(img, (3, 3))
imshow("blur", img2)

img3 = Canny(img, 10, 200)
imshow("canny", img3)

# selecting a region of interest
print img[10:20,10:20,0]

#computing the mean value for B channel
print numpy.mean(img[:, :, 0])
print numpy.mean(img[:, :, 1])
print numpy.mean(img[:, :, 2])

waitKey(0)
destroyAllWindows()
