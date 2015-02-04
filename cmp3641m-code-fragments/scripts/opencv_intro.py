from cv2 import namedWindow, imread, imshow
from cv2 import waitKey, destroyAllWindows, startWindowThread
from cv2 import blur, Canny

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


waitKey(0)
destroyAllWindows()
