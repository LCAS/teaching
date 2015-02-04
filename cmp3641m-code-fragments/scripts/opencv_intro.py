
from cv2 import namedWindow, imread, imshow, waitKey
from cv2 import destroyAllWindows, blur, Canny, startWindowThread

namedWindow("blur")
namedWindow("canny")

startWindowThread()

img = imread('../blofeld.jpg')
img2 = blur(img, (3, 3))
imshow("blur", img2)

img3 = Canny(img, 10, 200)
imshow("canny", img3)


waitKey(0)
destroyAllWindows()
