
import cv2

import numpy as np

# declare windows you want to display
cv2.namedWindow("original")
cv2.namedWindow("blur")
cv2.namedWindow("canny")

img = cv2.imread('blofeld.jpg')
print('type: %s' % type(img))
cv2.imshow("original", img)

# create a new blurred image:
blur_img = cv2.blur(img, (7, 7))

# draw on the image:
cv2.circle(blur_img, (100, 100), 30, (255, 0, 255), 5)

# display the image:
cv2.imshow("blur", blur_img)

# Canny is an algorith for edge detection
canny_img = cv2.Canny(img, 10, 200)
cv2.imshow("canny", canny_img)
print('shape: %s' % str(canny_img.shape))

# the shape gives you the dimensions
h = canny_img.shape[0]  # height
w = canny_img.shape[1]  # width

# loop over the image, pixel by pixel
count = 0
# a slow way to iterate over the pixels
for y in range(0, h):
    for x in range(0, w):
        # threshold the pixel
        if canny_img[y, x] > 0:
            count += 1
print('count edge pixels: %d' % count)

# a fast way to iterate using numpy:
count = np.sum(canny_img > 0)
print('faster count edge pixels: %d' % count)

# qait key is also always needed to sync the GUI threads
cv2.waitKey(0)
# good practice to tidy up at the end
cv2.destroyAllWindows()