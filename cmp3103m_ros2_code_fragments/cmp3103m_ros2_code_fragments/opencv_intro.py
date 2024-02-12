import cv2
import numpy as np

class ImageProcessor:
    def __init__(self, image_path):
        self.image_path = image_path
        self.img = None
        self.blur_img = None
        self.canny_img = None

    def initialize_windows(self):
        """Initialize the windows for displaying images."""
        cv2.namedWindow("original")
        cv2.namedWindow("blur")
        cv2.namedWindow("canny")

    def load_and_display_image(self):
        """Load an image from a file and display it."""
        self.img = cv2.imread(self.image_path)
        print('type: %s' % type(self.img))
        cv2.imshow("original", self.img)

    def blur_and_display_image(self):
        """Blur an image and display it."""
        self.blur_img = cv2.blur(self.img, (7, 7))
        cv2.circle(self.blur_img, (100, 100), 30, (255, 0, 255), 5)
        cv2.imshow("blur", self.blur_img)

    def apply_canny_and_display_image(self):
        """Apply the Canny algorithm for edge detection and display the image."""
        self.canny_img = cv2.Canny(self.img, 10, 200)
        cv2.imshow("canny", self.canny_img)
        print('shape: %s' % str(self.canny_img.shape))

    def count_edge_pixels(self):
        """Count the number of edge pixels in an image."""
        h = self.canny_img.shape[0]  # height
        w = self.canny_img.shape[1]  # width
        count = np.sum(self.canny_img > 0)
        print('faster count edge pixels: %d' % count)

    def process_image(self):
        self.initialize_windows()
        self.load_and_display_image()
        self.blur_and_display_image()
        self.apply_canny_and_display_image()
        self.count_edge_pixels()
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    processor = ImageProcessor('blofeld.jpg')
    processor.process_image()