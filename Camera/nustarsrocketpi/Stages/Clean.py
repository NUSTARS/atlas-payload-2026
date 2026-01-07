from Stage import Stage
from PIL.Image import Image   # ✅ import the class, not the module

class Clean(Stage):
    def open(self): pass
    def close(self): pass

    def process(self, image: Image) -> Image:
        pass