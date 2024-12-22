#!/usr/bin/env python
# Made by Tyler Jacques FRC Team 2648
# https://gitlab.coldlightalchemist.com/Tyler-J42/apriltag-pose-frc

import time
import cv2
import dt_apriltags
import numpy as np
from math import sqrt
from math import pi
import argparse
from Picam2Vid import Picam2Vid
from TagPin import TagPin
import constants
import ntcore

RAD2DEG = 180*pi

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
#parser.add_argument("--high_res", action='store_true', help="enable resolution 1088x720 vs 640x480")
parser.add_argument("--ip_add", type=str, required=True)
args = parser.parse_args()

'''
if args.high_res:
    FOCAL_LEN_PIXELS = 991.5391539
    camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0.00000000, 528.420369],
    [0.00000000, FOCAL_LEN_PIXELS, 342.737594],
    [0.00000000, 0.00000000, 1.00000000]])
    dist = np.array([[ 2.52081760e-01, -1.34794418e+00,  1.24975695e-03, -7.77510823e-04,
    2.29608398e+00]])
    constants.camera_res = (1088, 720)
'''

data_array = []

# network tables + RoboRio IP
inst = ntcore.NetworkTableInstance.getDefault()
vision_table = inst.getTable("Fiducial")
tag3tx = vision_table.getDoubleTopic("tag3tx").publish()
tag3ty = vision_table.getDoubleTopic("tag3ty").publish()
FPS_topic = vision_table.getDoubleTopic("fps").publish()
inst.startClient4("client")
inst.setServerTeam(2648)

FPS = 0

testing_tags = [[0, 0.0, 0.0, 0.0, 0.0], [1, 12.0, 0.0, 0.0, 0.0], [2, -12.0, 0.0, 0.0, 0.0]]

# x,y,z,rx,ry,rz
robo_space_pose = [0, 0, 0, 0, 0, 0]

cam = cv2.VideoCapture(0)

cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

cam.set(cv2.CAP_PROP_FPS, 100.0)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # auto mode
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual mode
cam.set(cv2.CAP_PROP_EXPOSURE, 30)

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)


# create overlay on camera feed
def display_features(image, tx, ty):
    # making red lines around fiducial
    for i in range(0,4):
        f = i+1
        if f>3: f=0
        cv2.line(image, (int(det.corners[i][0]), int(det.corners[i][1])), (int(det.corners[f][0]), int(det.corners[f][1])), (0,0,255), 3)

    image = cv2.putText(image, "#"+str(det.tag_id)+", "+str(int(tx))+", "+str(int(ty)), (int(det.center[0]),int(det.center[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
    return image

def findAngle(hor_coord, vert_coord):
    centerX = constants.camera_res[0]/2
    centerY = constants.camera_res[1]/2

    hor_angle = (hor_coord - centerX) / centerX * (constants.HORIZONTAL_FOV/2)
    vert_angle = -(vert_coord - centerY) / centerY * (constants.VERTICAL_FOV/2)

    return hor_angle, vert_angle

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
    image_corners = np.array([])
    tags_detected = []
    data_array = []

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

            hor_angle, vert_angle = findAngle(det.center[0], det.center[1])

            # only show display if you use --display for argparse
            if args.display:
                tag_points = np.array([[det.center[0], det.center[1]], [det.corners[0][0], det.corners[0][1]], [det.corners[1][0], det.corners[1][1]], [det.corners[2][0], det.corners[2][1]], [det.corners[3][0], det.corners[3][1]]], dtype=np.float32)
                image = display_features(image, hor_angle, vert_angle)

            data_array.append(TagPin(det.tag_id, hor_angle, vert_angle))
    
    if(len(tags_detected) > 0):

        for i in range(len(data_array)):
            if(data_array[i].tag_id == 3):
                tag3tx.set(data_array[i].get_tx())
                tag3ty.set(data_array[i].get_ty())
    else:
        tag3tx.set(0.0)
        tag3ty.set(0.0)
        
    FPS_topic.set(FPS)
    
    #Showing image. use --display to show image
    if args.display:
        image = cv2.putText(image, "FPS: "+str(round(FPS, 4)), (25,440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv2.LINE_AA)
        cv2.imshow("Frame", image)

        key = cv2.waitKey(1) & 0xFF
        if key ==ord("q"):
            break

    counter = counter+1
    if(counter==25):
        # frame rate for performance
        FPS = (1/(time.time()-frame_start))
        counter = 0
        #print(FPS)

    #vision_table.putNumber("FPS", FPS)

cam.stop()
cv2.destroyAllWindows()