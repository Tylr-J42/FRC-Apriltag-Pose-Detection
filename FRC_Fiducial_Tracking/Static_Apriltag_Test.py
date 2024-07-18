import dt_apriltags
import numpy as np
import cv2

detector = dt_apriltags.Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=2,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=1)

image = cv2.imread("/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/bw_img.png")

tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

print(tagFrame.dtype)

cv2.imshow("frame", image)
cv2.waitKey(0)

output = detector.detect(tagFrame, estimate_tag_pose=False, camera_params=None, tag_size=None)

print(output)