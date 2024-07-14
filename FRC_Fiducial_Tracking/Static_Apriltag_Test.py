import apriltag
import numpy as np
import cv2

options = apriltag.DetectorOptions(families='tag36h11', border=1, nthreads=1,
quad_decimate=1.0, quad_blur=0.0, refine_edges=True,
refine_decode=False, refine_pose=False, debug=False, quad_contours=True)
detector = apriltag.Detector(options)

image = cv2.imread("/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/0.jpg")

tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

output = detector.detect(tagFrame)

print(output)