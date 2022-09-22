# Made by Tyler Jacques FRC Team 2648
# Before deployment in competition comment out lines: 88, 89, and 97
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import apriltag
import numpy as np
import math
from networktables import NetworkTables


# translation vector units to inches tvec/71.22
TVEC2IN = 1/71.22
# Rotational vector radians to degrees
RAD2DEG = 180/math.pi

# RPi camera recording setup.
#  For specs - https://www.raspberrypi.com/documentation/accessories/camera.html
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))

# focal length in pixels. You can calculate using camera spec sheet for more accuracy
FOCAL_LEN_PIXELS = 528.6956522
# camera matrix from Calibrate_Camera.py.
camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0., 308.94165115],
 [0., FOCAL_LEN_PIXELS, 221.9470321],
 [0., 0.,1.]])

# 3d object array
objp = np.array([[240, 240, 0], [0, 0, 0], [480, 0, 0], [480, 480, 0], [0, 480, 0]], dtype=np.float32)
# 2d image array
axis = np.array([[0,0,0], [0,480,0], [480,480,0], [480,0,0], [0,0,-480], [0,480,-480], [480,480,-480], [480,0,-480]], dtype=np.float32)

NetworkTables.initialize(server="1234567890")
vision_table = NetworkTables.getTable("Fiducial")

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

def display_features(image, imgpts):
    # making lines on fiducial
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
    return image

time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame_start = time.time()
    image = frame.array

    #detecting april tags
    tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = apriltag.Detector()
    output = detector.detect(tagFrame)

    for det in output:
        
        tag_points = np.array([[det.center[0], det.center[1]], [det.corners[0][0], det.corners[0][1]], [det.corners[1][0], det.corners[1][1]], [det.corners[2][0], det.corners[2][1]], [det.corners[3][0], det.corners[3][1]]], dtype=np.float32)

        dist = np.array([ 2.32929183e-01, -1.35534844e+00, -1.51912733e-03, -2.17960810e-03, 2.25537289e+00])

        ret,rvecs, tvecs = cv2.solvePnP(objp, tag_points, camera_matrix, dist, flags=0)

        tvecDist = (tvecs*TVEC2IN).tolist()
        rvecDeg = (rvecs*RAD2DEG).tolist()
        for i in range(0,len(tvecDist)):
            tvecDist[i] = float(tvecDist[i][0])
        for i in range(0,len(rvecDeg)):
            rvecDeg[i] = float(rvecDeg[i][0])

        # comment out bottom two lines to eliminate drawing overlay
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist)
        image = display_features(image, imgpts)
        
        #print("tag"+str(det.tag_id)+"tvecs", tvecDist)
        #print("tag"+str(det.tag_id)+"rvecs", rvecDeg)
        vision_table.putNumberArray("tag"+str(det.tag_id)+"tvecs", tvecDist)
        vision_table.putNumberArray("tag"+str(det.tag_id)+"rvecs", rvecDeg)

    #Showing image. comment to stop display and speed up detection
    cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key ==ord("q"):
        break

    # frame rate for performance
    FPS = (1/(time.time()-frame_start))
    #print(FPS)
