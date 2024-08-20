#!/usr/bin/env python
# Made by Tyler Jacques FRC Team 2648
# https://gitlab.coldlightalchemist.com/Tyler-J42/apriltag-pose-frc

import time
import cv2
import dt_apriltags
import numpy as np
from math import sqrt
from math import pi
import math
from networktables import NetworkTables
import argparse
from TagObj import TagObj
from Picam2Vid import Picam2Vid
from PNP_Pose_Estimation import PNPPose

RAD2DEG = 180*pi

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
#parser.add_argument("--high_res", action='store_true', help="enable resolution 1088x720 vs 640x480")
parser.add_argument("--pose_estimation", action='store_true', help="estimate pose based on detected tags")
parser.add_argument("--ip_add", type=str, required=True)
args = parser.parse_args()

# focal length in pixels. You can use Camera_Calibrate.py and take at least 10 pics of a chess board or calculate using a camera spec sheet
# focal_length [mm] / imager_element_length [mm/pixel]
# 621.5827338
#FOCAL_LEN_PIXELS = 621.5827338
# camera matrix from Calibrate_Camera.py.
camera_matrix = np.array([[976.16482142,   0.,         771.05155174],
                        [  0.,         974.47104393, 408.52081949],
                        [  0.,           0.,           1.        ]])

# from Camera_Calibration.py
dist = np.array([-0.04790604,  0.08489533, -0.00387366,  0.00616192, -0.03875398])
 
camera_res = (1536, 864)

'''
if args.high_res:
    FOCAL_LEN_PIXELS = 991.5391539
    camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0.00000000, 528.420369],
    [0.00000000, FOCAL_LEN_PIXELS, 342.737594],
    [0.00000000, 0.00000000, 1.00000000]])
    dist = np.array([[ 2.52081760e-01, -1.34794418e+00,  1.24975695e-03, -7.77510823e-04,
    2.29608398e+00]])
    camera_res = (1088, 720)
'''

data_array = []

pose_coords = []
z_line_offset = 0

b=6.5
# 3d object array. The points of the 3d april tag that coresponds to tag_points which we detect
objp = np.array([[0,0,0], [-b/2, -b/2, 0], [b/2, -b/2, 0], [b/2, b/2, 0], [-b/2, b/2, 0]], dtype=np.float32)
# 2d axis array points for drawing cube overlay
axis = np.array([[b/2, b/2, 0], [-b/2, b/2, 0], [-b/2, -b/2, 0], [b/2, -b/2, 0], [b/2, b/2, b], [-b/2, b/2, b], [-b/2, -b/2, b], [b/2, -b/2, b]], dtype=np.float32)

# network tables + RoboRio IP
NetworkTables.initialize(server=args.ip_add)
vision_table = NetworkTables.getTable("Fiducial")

FPS = 0
        
# 2023 Field Apriltag Coordinates index = tag id
# format = [id, x, y, z, z-rotation] in inches
tag_coords = [[0, 0.0, 0.0, 0.0, 0.0], [1, 610.77, 42.19, 18.22, 180], [2, 610.77, 108.19, 18.22, 180], [3, 610.77, 174.19, 18.22, 180],
[4, 636.96, 265.74, 27.38, 180], [5, 14.25, 265.74, 27.38, 0], [6, 40.45, 174.19, 18.22, 0], [7, 40.45, 108.19, 18.22, 0],
[8, 40.45, 42.19, 18.22, 0]]

testing_tags = [[0, 0.0, 0.0, 0.0, 0.0], [1, 12.0, 0.0, 0.0, 0.0], [2, -12.0, 0.0, 0.0, 0.0]]

# x,y,z,rx,ry,rz
robo_space_pose = [0, 0, 0, 0, 0, 0]

def tag_corners(tag_coords):
    corners = []

    for i in range(len(tag_coords)):
        x = tag_coords[i][1]
        y = tag_coords[i][2]
        z = tag_coords[i][3]
        y_rotation = tag_coords[i][4]

        coordinates = [[], [], [] ,[], []]

        x_offset = (b/2)*math.cos(math.radians(y_rotation))
        z_offset = (b/2)*math.sin(math.radians(y_rotation))
        coordinates[0] = tag_coords[i][0]
        
        coordinates[1] = [x-x_offset, y+b/2, z+z_offset]
        coordinates[2] = [x+x_offset, y+b/2, z+z_offset]
        coordinates[3] = [x+x_offset, y-b/2, z+z_offset]
        coordinates[4] = [x-x_offset, y-b/2, z+z_offset]

        corners = corners + [coordinates]
    return corners

#field_tag_coords = tag_corners(tag_coords)
testing_tag_coords = tag_corners(testing_tags)

def getTagCoords(tag_id):
    return tag_coords[tag_id]

cam = Picam2Vid(camera_res)

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

# create overlay on camera feed
def display_features(image, imgpts):
    # making red lines around fiducial
    for i in range(0,4):
        f = i+1
        if f>3: f=0
        cv2.line(image, (int(det.corners[i][0]), int(det.corners[i][1])), (int(det.corners[f][0]), int(det.corners[f][1])), (0,0,255), 3)

    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    #image = cv2.drawContours(image, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        image = cv2.line(image, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    image = cv2.drawContours(image, [imgpts[4:]],-1,(0,0,255),3)
    image = cv2.putText(image, "#"+str(det.tag_id)+", "+"in", (int(det.center[0]),int(det.center[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
    return image

# setting up apriltag detection. Make sure this is OUTSIDE the loop next time
detector = dt_apriltags.Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=3,
                       quad_decimate=2,
                       quad_sigma=0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

pose_estimator = PNPPose(testing_tag_coords, robo_space_pose, camera_matrix, dist)

counter = 0

# main vision processing code
time.sleep(0.1)
while True:
    frame_start = time.time()
    cam.update()
    image = cam.read()
    image_corners = np.array([])
    tags_detected = []

    #detecting april tags
    tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    output = detector.detect(tagFrame)

    for det in output:
        # if the confidence is less than 30% exclude the tag from being processed.
        if det.decision_margin>30:
            # points of the tag to be tracked
            image_corners = list(image_corners)
            image_corners = image_corners+(list(det.corners))
            image_corners = np.array(image_corners)
            tags_detected.append(det.tag_id)

            # only show display if you use --display for argparse
            if args.display:
                tag_points = np.array([[det.center[0], det.center[1]], [det.corners[0][0], det.corners[0][1]], [det.corners[1][0], det.corners[1][1]], [det.corners[2][0], det.corners[2][1]], [det.corners[3][0], det.corners[3][1]]], dtype=np.float32)
                ret,rvecs, tvecs = cv2.solvePnP(objp, tag_points, camera_matrix, dist, flags=0)
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist)
                image = display_features(image, imgpts)

    if(len(tags_detected) > 0):
        pose_coords = pose_estimator.calculate_coords(image_corners, tags_detected)

        vision_table.putNumber("global_pose_x", pose_coords[0])
        vision_table.putNumber("global_pose_y", pose_coords[1])
        vision_table.putNumber("global_pose_z", pose_coords[2])

        vision_table.putNumberArray("visibleTags", tags_detected)
    
    #Showing image. use --display to show image
    if args.display:
        image = cv2.putText(image, "FPS: "+str(round(FPS, 4)), (25,440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv2.LINE_AA)
        cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF
    if key ==ord("q"):
        break

    # frame rate for performance
    FPS = (1/(time.time()-frame_start))

    counter = counter+1
    if(counter%10):
        print(FPS)

    vision_table.putNumber("FPS", FPS)

cam.stop()
cv2.destroyAllWindows()