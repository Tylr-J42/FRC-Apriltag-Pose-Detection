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


# translation vector units to inches: tvec/71.22 this constant will differ
# according to your camera. Space an apriltag at intervals, note the distance
# in pixels and divide it by the real world distance
TVEC2IN = 1
# Rotational vector radians to degrees
RAD2DEG = 180/pi


# focal length in pixels. You can use Camera_Calibrate.py or calculate using a camera spec sheet for more accuracy
# focal_length [mm] / imager_element_length [mm/pixel]
FOCAL_LEN_PIXELS = 528.6956522
# camera matrix from Calibrate_Camera.py.
camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0., 308.94165115],
 [0., FOCAL_LEN_PIXELS, 221.9470321],
 [0., 0.,1.]])

b=6.5
# 3d object array. The points of the 3d april tag that coresponds to tag_points which we detect
objp = np.array([[0,0,0], [b/2, b/2, 0], [-b/2, b/2, 0], [-b/2, -b/2, 0], [b/2, -b/2, 0]], dtype=np.float32)
# 2d axis array points for drawing cube overlay
axis = np.array([[b/2, b/2, 0], [-b/2, b/2, 0], [-b/2, -b/2, 0], [b/2, -b/2, 0], [b/2, b/2, -b], [-b/2, b/2, -b], [-b/2, -b/2, -b], [b/2, -b/2, -b]], dtype=np.float32)

# put your RoboRio IP here
NetworkTables.initialize(server="1234567890")
vision_table = NetworkTables.getTable("Fiducial")

# To show display of camera feed add --display in terminal when running script.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
args = parser.parse_args()

FPS = 0

TARGET_ID = 1

# class for allocating a thread to only updating the camera stream,
# the other thread is used for detection processing
class PiVid:

    def __init__(self):
        # RPi camera recording setup with threading crap.
        #  For specs - https://www.raspberrypi.com/documentation/accessories/camera.html
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 60
        self.rawCapture = PiRGBArray(self.camera, size=(640,480))
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
    # image = cv2.putText(image, str((round(tvecDist[0], 4), round(tvecDist[1], 4), round(tvecDist[2], 4))), (int(det.center[0])+100,int(det.center[1])+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
    # image = cv2.putText(image, str((round(rvecDeg[0], 4), round(rvecDeg[1], 4), round(rvecDeg[2], 4))), (int(det.center[0])+100,int(det.center[1])+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
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
        # from Camera_Calibration.py
        dist = np.array([ 2.32929183e-01, -1.35534844e+00, -1.51912733e-03, -2.17960810e-03, 2.25537289e+00])

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
        

    # writing data to networktables and ordering tags
    target_detected = False

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

        if TARGET_ID == data_array[0]:
            target_detected = True

    vision_table.putBoolean("targetDetected", target_detected)

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