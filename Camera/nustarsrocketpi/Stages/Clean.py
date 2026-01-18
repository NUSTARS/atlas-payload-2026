from Stage import Stage
from PIL import Image, ImageFilter  # ✅ import the class, not the module

class Clean(Stage):
    def open(self): 
        self.filter =  ImageFilter.MedianFilter(3)

    def close(self): 
        self.filter = None

    def process(self, image: Image.Image) -> Image.Image:
        return image.convert("L").filter(self.filter) 