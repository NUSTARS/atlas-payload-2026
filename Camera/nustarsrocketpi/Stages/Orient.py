from Stage import Stage
from PIL.Image import Image   # ✅ import the class, not the module

# Notes:
#   Self is an instance of the Orient class. I can access the data within the constructor 
#   ': list' in the constructor is hinting at the fact that it should be a list but is not enforced (wft is this, why is python so goofy)
#   Some good resources:
#       - https://pillow.readthedocs.io/en/stable/handbook/tutorial.html


# Eliott 
class Orient(Stage):
    # IMU data: {"orient": [], "gravity": [], ...}
    # Let's assume the data has a roll_z component in it
    def __init__(self, IMU_data: list, altimeter_data: list): # underscore is for python to know that its a constructor. Why :list???
        self.IMU_data = IMU_data
        self.Alt_data = altimeter_data


    # What do these open and close functions do again??
    def open(self): pass
    def close(self): pass

    #
    # Objective: Look into libraries to see how to orient the image given stuff into the constructor 
    # 

    # This process function will take in a list of IMU data and an image and return a rotated image based on the offset of the IMU data. 
    def process(self, image: Image):
        # To orient the image properly we can do the following:
        #   - We need a 0 point to rotate all of the images to. This can be selected in the following ways: 
        #       We orient everything based on the first frame. We orient everything based on the 'true' zero of the payload
        #   - We assume we have an updated key pair, which is a made up word that I will define as {image, IMU_data}. 
        #       I WILL ALSO ASSUME THAT THE roll_z DATA IS IN THE RANGE [0, 360] WHERE IT INCREASES GOING CLOCKWISE 
        #       (BUT THIS IS NOT ALWAYS THE CASE). [-180, 180] is also possible.
        #       We extract the key pair.
        #   - We now need to choose an arbirary direction to rotate the images. This MUST be consistent with all of the images.
        #       I am going to choose to rotate ALL images clockwise. The rotation factor is determined by the difference of 
        #       roll_z and 360 degrees. For example, if I am given an image that has a key pair of {image, 70}, this means that 
        #       I need to rotate the image (360 - 70) degrees clockwise. 
        #   - Lastly, find a suitable library that can take in an image and rotate it based on degrees or radians. 
        
        angleCW = 360 - self.IMU_data
        image = image.rotate(angleCW) # Assuming we are using PIL library

        return image
