import numpy as np

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
        bgr = np.random.randint(0, 20, 3)
        image = np.zeros((480, 640, 3))
        image[:, :] = bgr
        return image
    
