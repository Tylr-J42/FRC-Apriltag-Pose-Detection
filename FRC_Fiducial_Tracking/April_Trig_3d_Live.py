#!/usr/bin/env python
# Made by Tyler Jacques FRC Team 2648
# https://gitlab.coldlightalchemist.com/Tyler-J42/apriltag-pose-frc

from os import close
import time
import cv2
import dt_apriltags
import numpy as np
from math import sqrt
from math import pi
import math
import argparse

from soupsieve import closest
import constants
import ntcore

from wpimath.geometry import Translation3d
from wpimath.geometry import Transform3d
from wpimath.geometry import Pose3d
from wpimath.geometry import Rotation3d

RAD2DEG = 180*pi

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
#parser.add_argument("--high_res", action='store_true', help="high resolution camera capture")
parser.add_argument("--pose_estimation", action='store_true', help="estimate pose based on detected tags")
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

tx = 0
ty = 0
dist = 0
tracked_tag = 0

FPS = 0

inst = ntcore.NetworkTableInstance.getDefault()

if constants.camera_color == "orange":
    # network tables + RoboRio IP
    vision_table = inst.getTable("orange_Fiducial")

    tag_topic = vision_table.getDoubleTopic("orangeClosestTag").publish()
    tx_topic = vision_table.getDoubleTopic("tx").publish()
    ty_topic = vision_table.getDoubleTopic("ty").publish()
    total_dist_topic = vision_table.getDoubleTopic("totalDist").publish()

    is_tag_detected = vision_table.getBooleanTopic("orangeTagDetected").publish()

    FPS_topic = vision_table.getDoubleTopic("orangeFPS").publish()

    
elif constants.camera_color == "black":
    # network tables + RoboRio IP
    vision_table = inst.getTable("black_Fiducial")

    tag_topic = vision_table.getDoubleTopic("blackClosestTag").publish()
    tx_topic = vision_table.getDoubleTopic("tx").publish()
    ty_topic = vision_table.getDoubleTopic("ty").publish()
    total_dist_topic = vision_table.getDoubleTopic("totalDist").publish()

    is_tag_detected = vision_table.getBooleanTopic("blackTagDetected").publish()

    FPS_topic = vision_table.getDoubleTopic("blackFPS").publish()

inst.startClient4("client")
inst.setServerTeam(2648)


# set camera parameters
cam = cv2.VideoCapture(0)

cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
cam.set(cv2.CAP_PROP_FPS, 100.0)
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual mode
cam.set(cv2.CAP_PROP_EXPOSURE, 60)
cam.set(cv2.CAP_PROP_SHARPNESS, 3)
cam.set(cv2.CAP_PROP_BRIGHTNESS, 0)
cam.set(cv2.CAP_PROP_CONTRAST, 32)

'''
cam = cv2.VideoCapture(2)

cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
cam.set(cv2.CAP_PROP_FPS, 100.0)
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual mode
cam.set(cv2.CAP_PROP_EXPOSURE, 60)
cam.set(cv2.CAP_PROP_SHARPNESS, 3)
cam.set(cv2.CAP_PROP_BRIGHTNESS, 0)
cam.set(cv2.CAP_PROP_CONTRAST, 32)
'''

def findTagAngle(point):
    undistorted_pointXY = cv2.undistortPoints(point, constants.camera_matrix, constants.dist, None, constants.camera_matrix, )[0][0]

    center_vector = np.linalg.inv(constants.camera_matrix).dot(np.array([undistorted_pointXY[0], undistorted_pointXY[1], 1]).T)

    horizontal_angle = np.rad2deg(math.atan(center_vector[0]))
    vertical_angle = np.rad2deg(math.atan(center_vector[1]))
    #print("horizontal angle: " + str(horizontal_angle))
    #print("vertical angle: " + str(vertical_angle))

    return (180, vertical_angle, horizontal_angle)

def trig_3d_solver(center_coords, dist_3d):
    xyz_angle = findTagAngle(center_coords)

    robot_camera = Pose3d(Translation3d(constants.cam_orange_robo_pose[3], constants.cam_orange_robo_pose[4], constants.cam_orange_robo_pose[5]),
                           Rotation3d(np.deg2rad(constants.cam_orange_robo_pose[0]), np.deg2rad(constants.cam_orange_robo_pose[1]), np.deg2rad(constants.cam_orange_robo_pose[2])))
    
    robot_tag_pose = robot_camera.transformBy(Transform3d(Translation3d(dist_3d, 0, 0), Rotation3d(np.rad2deg(xyz_angle[0]), np.rad2deg(xyz_angle[1]), np.rad2deg(xyz_angle[2]))))

    return robot_tag_pose.X(), robot_tag_pose.Y(), robot_tag_pose.Z()

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
                       decode_sharpening=0.0,
                       debug=0)

counter = 0

# main vision processing code
time.sleep(0.1)
while True:
    frame_start = time.time()
    ret, image = cam.read()
    data_array = []
    tags_detected = []
    tx = 0
    ty = 0
    dist = 0

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

            data_array.append([det.tag_id, totalDist, det.center])
            tags_detected.append(det.tag_id)

    if(len(data_array) == 1):
        #robot_tvec_x, robot_tvec_y, robot_tvec_z = trig_3d_solver(data_array[0][2], data_array[0][1])
        fein, ty, tx = findTagAngle(data_array[0][2])
        dist = data_array[0][1]
        tracked_tag = data_array[0][0]
    elif(len(data_array)>1):
        closest_tag = 0
        largest_dist = 0
        # sort for the closest tag
        for i in range(len(data_array)):
            if(data_array[i][1] > largest_dist):
                closest_tag = i
        
        #robot_tvec_x, robot_tvec_y, robot_tvec_z = trig_3d_solver(data_array[closest_tag][2], data_array[closest_tag][1])
        fein, ty, tx = findTagAngle(data_array[closest_tag][2])
        dist = data_array[closest_tag][1]
        tracked_tag = data_array[closest_tag][0]

    #Showing image. use --display to show image
    if args.display:
        image = cv2.putText(image, "FPS: "+str(round(FPS, 4)), (25,440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv2.LINE_AA)
        cv2.imshow("Frame", image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    tag_topic.set(tracked_tag)
    tx_topic.set(tx)
    ty_topic.set(ty)
    total_dist_topic.set(dist)
    inst.flush()

   # print("robot tvec:", robot_tvec_x, robot_tvec_y, robot_tvec_z)

    FPS_topic.set(FPS)

    is_tag_detected.set(len(tags_detected) > 0)

    counter = counter+1
    if(counter==25):
        # frame rate for performance
        FPS = (1/(time.time()-frame_start))
        counter = 0
        print(FPS)