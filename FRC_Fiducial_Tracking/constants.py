import numpy as np

class constants:

    def __init__(self):
        self.FOCAL_LEN_PIXELS = 621.5827338
        self.camera_matrix = np.array([[976.16482142,   0.,         771.05155174],
                        [  0.,         974.47104393, 408.52081949],
                        [  0.,           0.,           1.        ]])
        
        # 2023 Field Apriltag Coordinates index = tag id
        # format = [id, x, y, z, z-rotation] in inches
        self.tag_coords = [[0, 0.0, 0.0, 0.0, 0.0], [1, 610.77, 42.19, 18.22, 180], [2, 610.77, 108.19, 18.22, 180], [3, 610.77, 174.19, 18.22, 180],
        [4, 636.96, 265.74, 27.38, 180], [5, 14.25, 265.74, 27.38, 0], [6, 40.45, 174.19, 18.22, 0], [7, 40.45, 108.19, 18.22, 0],
        [8, 40.45, 42.19, 18.22, 0]]

        # from Camera_Calibration.py
        self.dist = np.array([-0.04790604,  0.08489533, -0.00387366,  0.00616192, -0.03875398])

        self.VERTICAL_FOV = 67
        self.HORIZONTAL_FOV = 102
        