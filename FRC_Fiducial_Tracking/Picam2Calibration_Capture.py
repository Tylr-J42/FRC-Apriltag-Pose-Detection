from Picam2Vid import Picam2Vid
import cv2
import os

stream = Picam2Vid((2304, 1296))

path = "/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/"

stream.update()
frame = stream.read()
cv2.imshow("frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
confirmation = input("keep y or n: ")
print(frame)
if confirmation == "y":
    file_order = len(os.listdir(path))
    cv2.imwrite(path+str(file_order)+".jpg", frame)
stream.stop()
