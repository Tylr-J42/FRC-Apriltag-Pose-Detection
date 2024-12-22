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
import constants

from TagObj import TagObj
#from PiVid import PiVid
from Picam2Vid import Picam2Vid

RAD2DEG = 180*pi

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
#parser.add_argument("--high_res", action='store_true', help="high resolution camera capture")
parser.add_argument("--pose_estimation", action='store_true', help="estimate pose based on detected tags")
parser.add_argument("--ip_add", type=str, required=True)
args = parser.parse_args()

'''
if args.high_res:
    FOCAL_LEN_PIXELS = 991.5391539
    constants.camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0.00000000, 528.420369],
    [0.00000000, FOCAL_LEN_PIXELS, 342.737594],
    [0.00000000, 0.00000000, 1.00000000]])
    constants.dist = np.array([[ 2.52081760e-01, -1.34794418e+00,  1.24975695e-03, -7.77510823e-04,
    2.29608398e+00]])
    camera_res = (2304, 1296)
'''

b=6.5
# 3d object array. The points of the 3d april tag that coresponds to tag_points which we detect
objp = np.array([[0,0,0], [-b/2, -b/2, 0], [b/2, -b/2, 0], [b/2, b/2, 0], [-b/2, b/2, 0]], dtype=np.float32)
# 2d axis array points for drawing cube overlay
axis = np.array([[b/2, b/2, 0], [-b/2, b/2, 0], [-b/2, -b/2, 0], [b/2, -b/2, 0], [b/2, b/2, b], [-b/2, b/2, b], [-b/2, -b/2, b], [b/2, -b/2, b]], dtype=np.float32)

# network tables + RoboRio IP
NetworkTables.initialize(server=args.ip_add)
vision_table = NetworkTables.getTable("Fiducial")

FPS = 0

# x,y,z,rx,ry,rz
robo_space_pose = [0, 0, 0, 0, 0, 0]

def tag_corners(tag_coords):
    corners = []

    for i in range(len(constants.tag_coords)):
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

        corners.append(coordinates)

    return corners

field_tag_coords = tag_corners(constants.tag_coords)

def getTagCoords(tag_id):
    return constants.tag_coords[tag_id]

cam = Picam2Vid(constants.camera_res)

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

# create overlay on camera feed
def display_features(image, imgpts, totalDist):
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
    image = cv2.putText(image, "#"+str(det.tag_id)+", "+str(round(totalDist, 4))+"in", (int(det.center[0]),int(det.center[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
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

counter = 0

# main vision processing code
time.sleep(0.1)
while True:
    frame_start = time.time()
    cam.update()
    image = cam.read()
    data_array = []
    visible_tags = []

    #detecting april tags
    tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    output = detector.detect(tagFrame)

    #print(output)

    for det in output:
        # if the confidence is less than 30% exclude the tag from being processed.
        if det.decision_margin>30:
            # points of the tag to be tracked
            tag_points = np.array([[det.center[0], det.center[1]], [det.corners[0][0], det.corners[0][1]], [det.corners[1][0], det.corners[1][1]], [det.corners[2][0], det.corners[2][1]], [det.corners[3][0], det.corners[3][1]]], dtype=np.float32)

            ret,rvecs, tvecs = cv2.solvePnP(objp, tag_points, constants.camera_matrix, constants.dist, flags=0)

            # making translation and rotation vectors into a format good for networktables
            tvecDist = tvecs.tolist()
            rvecDeg = (rvecs*RAD2DEG).tolist()
            for i in range(0,len(tvecDist)):
                tvecDist[i] = float(tvecDist[i][0])
            for i in range(0,len(rvecDeg)):
                rvecDeg[i] = float(rvecDeg[i][0])

            totalDist = sqrt((tvecDist[0]**2)+(tvecDist[1]**2)+(tvecDist[2]**2))

            # only show display if you use --display for argparse
            if args.display:
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, constants.camera_matrix, constants.dist)
                image = display_features(image, imgpts, totalDist)

            data_array.append(TagObj(det.tag_id, tvecDist, rvecDeg, totalDist))

    for i in range(len(data_array)):

        vision_table.putNumber("tag"+str(data_array[i].tag_id)+"tvecX", data_array[i].tvec_x)
        vision_table.putNumber("tag"+str(data_array[i].tag_id)+"tvecY", data_array[i].tvec_y)
        vision_table.putNumber("tag"+str(data_array[i].tag_id)+"tvecZ", data_array[i].tvec_z)
        
        vision_table.putNumber("tag"+str(data_array[i].tag_id)+"rvecX", data_array[i].rvec_x)
        vision_table.putNumber("tag"+str(data_array[i].tag_id)+"rvecY", data_array[i].rvec_y)
        vision_table.putNumber("tag"+str(data_array[i].tag_id)+"rvecZ", data_array[i].rvec_z)

        visible_tags.append(data_array[i].tag_id)

    vision_table.putNumberArray("visibleTags", visible_tags)

    #Showing image. use --display to show image
    if args.display:
        image = cv2.putText(image, "FPS: "+str(round(FPS, 4)), (25,440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv2.LINE_AA)
        cv2.imshow("Frame", image)

        key = cv2.waitKey(1)

    # frame rate for performance
    FPS = (1/(time.time()-frame_start))

    counter = counter+1
    #if(counter%10):
        #print(FPS)

    vision_table.putNumber("FPS", FPS)

cam.stop()
cv2.destroyAllWindows()