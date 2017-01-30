import numpy as np
import cv2

class BaseImageProvider(object):
    '''Classes that provide image sources. Examples may include:
        - A bag crawler
        - A video crawler
        - Loading images from a folder
    '''
    name = "general image provider"

    def load_source(self, *args, **kwargs):
        ''' Load images from source '''
        pass

    def get_next_image(self):
        ''' Returns the next image in the sequence '''
        pass


class RandomColors(BaseImageProvider):
    name = "Random Color Generator"

    def get_next_image(self):
        ''' Generates random colors '''
        bgr = np.random.randint(0, 200, 3)
        image = np.zeros((480, 640, 3))
        image[:, :] = bgr
        return image


class Webcam(BaseImageProvider):
    name = "Webcam Capture"
    
    def load_source(self):
        self.cam = cv2.VideoCapture(0)

    def get_next_image(self):
        ret, frame = self.cam.read()
        return frame
