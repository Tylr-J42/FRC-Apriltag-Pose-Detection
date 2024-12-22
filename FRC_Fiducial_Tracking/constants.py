import numpy as np

# for the RPi v3 wide camera
'''
FOCAL_LEN_PIXELS = 621.5827338
camera_matrix = np.array([[976.16482142,   0.,         771.05155174],
                [  0.,         974.47104393, 408.52081949],
                [  0.,           0.,           1.        ]])
'''

#for the Arducam OV9872 Camera
camera_martix = np.array([[935.58552861,   0.,         664.73887135],
    [  0.,         932.27174304, 326.00933432],
    [  0.,           0.,           1.        ]])

# for arducam OV9872 camrea
dist = np.array([[-0.026375,    0.28978668, -0.00669531, -0.00973697, -0.48583437]])

# 2023 Field Apriltag Coordinates index = tag id
# format = [id, x, y, z, z-rotation] in inches
tag_coords = [[0, 0.0, 0.0, 0.0, 0.0], [1, 610.77, 42.19, 18.22, 180], [2, 610.77, 108.19, 18.22, 180], [3, 610.77, 174.19, 18.22, 180],
[4, 636.96, 265.74, 27.38, 180], [5, 14.25, 265.74, 27.38, 0], [6, 40.45, 174.19, 18.22, 0], [7, 40.45, 108.19, 18.22, 0],
[8, 40.45, 42.19, 18.22, 0]]

# from Camera_Calibration.py for RPi v3 wide camera
#dist = np.array([-0.04790604,  0.08489533, -0.00387366,  0.00616192, -0.03875398])

camera_res = (1280, 800)

# for RPi camera v3 wide
#camera_res = (1536, 864)

VERTICAL_FOV = 67
HORIZONTAL_FOV = 102

# for RPi Camera v3 Wide
#VERTICAL_FOV = 67
#HORIZONTAL_FOV = 102

# x,y,z,rx,ry,rz
robo_space_pose = [0, 0, 0, 0, 0, 0]        