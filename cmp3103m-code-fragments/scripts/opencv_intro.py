
from cv2 import namedWindow, imread, imshow
from cv2 import waitKey, destroyAllWindows, startWindowThread
from cv2 import blur, Canny, circle

# declare windows you want to display
namedWindow("original")
namedWindow("blur")
namedWindow("canny")

# this is always needed to run the GUI thread
startWindowThread()

img = imread('../blofeld.jpg')
imshow("original", img)
# create a new blurred image:
img2 = blur(img, (7, 7))
# draw on the image:
circle(img2, (100, 100), 30, (255, 0, 255), 5)
# display the image:
imshow("blur", img2)
# Canny is an algorith for edge detection
img3 = Canny(img, 10, 200)
imshow("canny", img3)
# the shape gives you the dimensions
h = img3.shape[0]
w = img3.shape[1]
# loop over the image, pixel by pixel
count = 0
# a slow way to iterate over the pixels
for y in range(0, h):
    for x in range(0, w):
        # threshold the pixel
        if img3[y, x] > 0:
            count += 1
print('count edge pixels: %d' % count)
# qait key is also always needed to sync the GUI threads
waitKey(0)
# good practice to tidy up at the end
destroyAllWindows()
