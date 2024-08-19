import dt_apriltags
import numpy as np
import cv2

FOCAL_LEN_PIXELS = 976.16482142
camera_matrix = np.array([[FOCAL_LEN_PIXELS,   0.,         771.05155174],
    [  0.,         FOCAL_LEN_PIXELS, 408.52081949],
    [  0.,           0.,           1.        ]])
    
dist = np.array([[-0.04790604,  0.08489533, -0.00387366,  0.00616192, -0.03875398]])

detector = dt_apriltags.Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=2,
                       quad_decimate=1,
                       quad_sigma=0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=1)

#image = cv2.imread("/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/apriltagrobots_overlay.jpg")
image = cv2.imread("/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/14.jpg")
#image = cv2.imread("/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/20240814_043333.jpg")

#h,  w = image.shape[:2]
#newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, (w,h), 1, (w,h))

# undistort
#dst = cv2.undistort(image, camera_matrix, dist, None, newcameramtx)
 
# crop the image
#x, y, w, h = roi
#dst = dst[y:y+h, x:x+w]

grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
smaller = cv2.resize(grey, (640, 360))

print(smaller)

cv2.imshow("frame", smaller)
cv2.waitKey(0)

output = detector.detect(smaller, estimate_tag_pose=True, camera_params=None, tag_size=None)

print(output)