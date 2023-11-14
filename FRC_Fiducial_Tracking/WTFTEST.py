import math
from PNP_Pose_Estimation import PNPPose
import numpy as np

FOCAL_LEN_PIXELS = 621.5827338
# camera matrix from Calibrate_Camera.py.
camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0., 308.94165115],
    [0., FOCAL_LEN_PIXELS, 221.9470321],
    [0., 0.,1.]])

# from Camera_Calibration.py
dist = np.array([ 2.32929183e-01, -1.35534844e+00, -1.51912733e-03, -2.17960810e-03, 2.25537289e+00])

# 2023 Field Apriltag Coordinates index = tag id
# format = [id, x, y, z, z-rotation] in inches
tag_coords = [[0, 69.0, 420.0, 69.0, 0.0], [1, 610.77, 42.19, 18.22, 180], [2, 610.77, 108.19, 18.22, 180], [3, 610.77, 174.19, 18.22, 180],
[4, 636.96, 265.74, 27.38, 180], [5, 14.25, 265.74, 27.38, 0], [6, 40.45, 174.19, 18.22, 0], [7, 40.45, 108.19, 18.22, 0],
[8, 40.45, 42.19, 18.22, 0]]

# x,y,z,rx,ry,rz
robo_space_pose = [0, 0, 0, 0, 0, 0]

b = 6.5

def tag_corners(tag_coords):
    corners = []

    for i in range(len(tag_coords)):
        x = tag_coords[i][1]
        y = tag_coords[i][2]
        z = tag_coords[i][3]
        z_rotation = tag_coords[i][4]

        coordinates = [[], [], [] ,[], []]

        x_offset = (b/2)*math.cos(math.radians(z_rotation))
        y_offset = (b/2)*math.sin(math.radians(z_rotation))
        coordinates[0] = tag_coords[i][0]
        coordinates[1] = [x-x_offset, y+y_offset, z+b/2]
        coordinates[2] = [x+x_offset, y+y_offset, z+b/2]
        coordinates[3] = [x+x_offset, y+y_offset, z-b/2]
        coordinates[4] = [x-x_offset, y+y_offset, z-b/2]

        corners = corners + [coordinates]
    return corners

field_tag_coords = tag_corners(tag_coords)
robo_space_pose = [0, 0, 0, 0, 0, 0]

pose_estimator = PNPPose(field_tag_coords, robo_space_pose, camera_matrix, dist)

pose_estimator.calculate_coords( np.array([[ 65.75, 420.,    72.25],
 [ 72.25, 420.,    72.25],
 [ 72.25, 420.,    65.75],
 [ 65.75, 420.,    65.75]]), np.array([[398.38015747, 154.20915222],
 [260.04067993, 150.67947388],
 [212.12528992 , 77.55628204],
 [387.06124878,  77.81692505]]))