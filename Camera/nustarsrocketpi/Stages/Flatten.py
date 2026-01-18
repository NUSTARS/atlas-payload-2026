from Stage import Stage
from PIL.Image import Image   # ✅ import the class, not the module

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# ASSUMPTIONS:
#  - IMU: z-axis down
#  - CAMERA: z-axis down

class Flatten(Stage):
    def open(self): pass
    def close(self): pass

    # IMU data is a list of format [yaw, pitch, roll]
    def process(self, image: Image, IMU_data: list) -> Image:

        # Convert image to numpy array (for transformation)
        n_image = np.asarray(image)

        # Extract IMU data
        yaw = IMU_data[0]
        pitch = IMU_data[1]
        roll = IMU_data[2]

        # Convert IMU data into matrix
        R_imu = R.from_euler(
            'ZYX',
            [yaw, pitch, roll],
            degrees=True
        ).as_matrix()

        # Alter matrix to match camera angle (TODO: ADJUST BASED ON PHYSICAL PARAMETERS)
        R_imu_to_cam = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        R_cam = R_imu_to_cam @ R_imu

        # Extract camera-frame angles
        r = R.from_matrix(R_cam)
        yaw_c, pitch_c, roll_c = r.as_euler('ZYX', degrees=False)

        # Remove tilt
        R_level = R.from_euler(
            'ZYX',
            [yaw_c, 0.0, 0.0]
        ).as_matrix()

        # Define camera intrinsics (TODO: MUST GET ACTUAL CAM VALUES)
        fx, fy = 800, 800 # DUMMY VALUES
        cx, cy = 640, 360 # DUMMY VALUES
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        # Calculate homography matrix
        H = K @ R_level @ np.linalg.inv(R_cam) @ np.linalg.inv(K)

        # Apply matrix to image
        h, w = n_image.shape[:2]
        leveled = cv2.warpPerspective(image, H, (w, h))

        # Convert leveled image to PIL and return
        leveled = Image.fromarray(leveled)
        return leveled