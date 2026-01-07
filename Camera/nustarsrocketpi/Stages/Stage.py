from abc import ABC, abstractmethod
from PIL.Image import Image

class Stage(ABC):

    @abstractmethod
    def open(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def process(self, image : Image) -> Image:
        pass

    