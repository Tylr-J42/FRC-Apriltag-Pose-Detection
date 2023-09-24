#!/usr/bin/env python
# Made by Tyler Jacques FRC Team 2648
# https://gitlab.coldlightalchemist.com/Tyler-J42/apriltag-pose-frc

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import apriltag
import numpy as np
from math import sqrt
from math import pi
from networktables import NetworkTables
import argparse
from threading import Thread


# Rotational vector radians to degrees
RAD2DEG = 180/pi

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
parser.add_argument("--high_res", action='store_true', help="enable resolution 1088x720 vs 640x480")
parser.add_argument("--ip_add", type=str, required=True)
args = parser.parse_args()

# focal length in pixels. You can use Camera_Calibrate.py and take at least 10 pics of a chess board or calculate using a camera spec sheet
# focal_length [mm] / imager_element_length [mm/pixel]
# 621.5827338
FOCAL_LEN_PIXELS = 621.5827338
# camera matrix from Calibrate_Camera.py.
camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0., 308.94165115],
 [0., FOCAL_LEN_PIXELS, 221.9470321],
 [0., 0.,1.]])

# from Camera_Calibration.py
dist = np.array([ 2.32929183e-01, -1.35534844e+00, -1.51912733e-03, -2.17960810e-03, 2.25537289e+00])
 
camera_res = (640, 480)

if args.high_res:
    FOCAL_LEN_PIXELS = 991.5391539
    camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0.00000000, 528.420369],
    [0.00000000, FOCAL_LEN_PIXELS, 342.737594],
    [0.00000000, 0.00000000, 1.00000000]])
    dist = np.array([[ 2.52081760e-01, -1.34794418e+00,  1.24975695e-03, -7.77510823e-04,
    2.29608398e+00]])
    camera_res = (1088, 720)

b=6.5
# 3d object array. The points of the 3d april tag that coresponds to tag_points which we detect
objp = np.array([[0,0,0], [b/2, b/2, 0], [-b/2, b/2, 0], [-b/2, -b/2, 0], [b/2, -b/2, 0]], dtype=np.float32)
# 2d axis array points for drawing cube overlay
axis = np.array([[b/2, b/2, 0], [-b/2, b/2, 0], [-b/2, -b/2, 0], [b/2, -b/2, 0], [b/2, b/2, -b], [-b/2, b/2, -b], [-b/2, -b/2, -b], [b/2, -b/2, -b]], dtype=np.float32)

# network tables + RoboRio IP
NetworkTables.initialize(server=args.ip_add)
vision_table = NetworkTables.getTable("Fiducial")

FPS = 0

TARGET_ID = 1

# 2023 Field Apriltag Coordinates index = tag id
# format = [x, y, z, z-rotation]
tag_coords = [0, [610.77, 42.19, 18.22, 180], [610.77, 108.19, 18.22, 180], [610.77, 174.19, 18.22, 180],
[636.96, 265.74, 27.38, 180], [14.25, 265.74, 27.38, 0], [40.45, 174.19, 18.22, 0], [40.45, 108.19, 18.22, 0],
[40.45, 42.19, 18.22, 0]]

def getTagCoords(tag_id):
    return tag_coords[tag_id]

# class for allocating a thread to only updating the camera stream,
# the other thread is used for detection processing
class PiVid:

    def __init__(self):
        # RPi camera recording setup with threading crap.
        #  For specs - https://www.raspberrypi.com/documentation/accessories/camera.html
        self.camera = PiCamera()
        self.camera.resolution = camera_res
        self.camera.framerate = 60
        self.rawCapture = PiRGBArray(self.camera, size=camera_res)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        self.frame = None
        self.stopped = False

    # Start camera thread
    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    # update camera stream threading
    def update(self):
        for frame in self.stream:
            self.frame=frame.array
            self.rawCapture.truncate(0)
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    # output the frame we want
    def read(self):
        return self.frame

    # end threading
    def stop(self):
        self.stopped = True

cam = PiVid().start()

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
    image = cv2.drawContours(image, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        image = cv2.line(image, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    image = cv2.drawContours(image, [imgpts[4:]],-1,(0,0,255),3)
    image = cv2.putText(image, "#"+str(det.tag_id)+", "+str(round(totalDist, 4))+"in", (int(det.center[0]),int(det.center[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
    return image

# setting up apriltag detection. Make sure this is OUTSIDE the loop next time
options = apriltag.DetectorOptions(families='tag36h11', border=1, nthreads=4,
quad_decimate=2.0, quad_blur=0.0, refine_edges=True,
refine_decode=False, refine_pose=False, debug=False, quad_contours=True)
detector = apriltag.Detector(options)

# main vision processing code
time.sleep(0.1)
while True:
    frame_start = time.time()
    image = cam.read()
    data_array = []

    #detecting april tags
    tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    output = detector.detect(tagFrame)

    for det in output:
        # points of the tag to be tracked
        tag_points = np.array([[det.center[0], det.center[1]], [det.corners[0][0], det.corners[0][1]], [det.corners[1][0], det.corners[1][1]], [det.corners[2][0], det.corners[2][1]], [det.corners[3][0], det.corners[3][1]]], dtype=np.float32)


        ret,rvecs, tvecs = cv2.solvePnP(objp, tag_points, camera_matrix, dist, flags=0)

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
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist)
            image = display_features(image, imgpts, totalDist)

        data_array.append([det.tag_id, tvecDist, rvecDeg, totalDist])
        

    for i in range(len(data_array)):
        orderVal = 0
        for d in range(len(data_array)):
            if data_array[d][2]>data_array[i][2] and d!=i and output[d].tag_id==output[i].tag_id:
                orderVal = ++orderVal

        vision_table.putNumber("tag"+str(data_array[i][0])+"tvecX("+str(orderVal)+")", tvecDist[0])
        vision_table.putNumber("tag"+str(data_array[i][0])+"tvecY("+str(orderVal)+")", tvecDist[1])
        vision_table.putNumber("tag"+str(data_array[i][0])+"tvecZ("+str(orderVal)+")", tvecDist[2])
        
        vision_table.putNumber("tag"+str(data_array[i][0])+"rvecX("+str(orderVal)+")", rvecDeg[0])
        vision_table.putNumber("tag"+str(data_array[i][0])+"rvecY("+str(orderVal)+")", rvecDeg[1])
        vision_table.putNumber("tag"+str(data_array[i][0])+"rvecZ("+str(orderVal)+")", rvecDeg[2])


    vision_table.putNumber("numberOfTags", len(data_array))

    #Showing image. use --display to show image
    if args.display:
        image = cv2.putText(image, "FPS: "+str(round(FPS, 4)), (25,440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv2.LINE_AA)
        cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF
    if key ==ord("q"):
        break

    # frame rate for performance
    FPS = (1/(time.time()-frame_start))
    #print(FPS)
    vision_table.putNumber("FPS", FPS)

cam.stop()
cv2.destroyAllWindows()