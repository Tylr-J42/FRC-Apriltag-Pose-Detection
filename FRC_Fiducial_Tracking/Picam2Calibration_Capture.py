from Picam2Vid import Picam2Vid
import cv2
import os

stream = Picam2Vid((1536, 864))

path = "/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/"

stream.update()
frame = stream.read()
cv2.imshow("frame", frame)
cv2.waitKey(2)
confirmation = input("keep y or n: ")
if confirmation == "y":
    file_order = len(os.listdir(path))
    cv2.imwrite(path+str(file_order)+".png", frame)
stream.stop()
