import cv2
import os
import time

stream = cv2.VideoCapture(0)

path = "/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Calibration/Calibration_Pics_OV9782/"

stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

print(stream.get(cv2.CAP_PROP_EXPOSURE))

stream.set(cv2.CAP_PROP_FPS, 100.0)
stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

def getFrame():
    ret, output = stream.read()
    if(ret):
        return output
    else:
        getFrame()

frame = getFrame()
cv2.imshow("frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
confirmation = input("keep y or n: ")
print(frame)
if confirmation == "y":
    file_order = len(os.listdir(path))
    cv2.imwrite(path+str(file_order)+".jpg", frame)
stream.stop()
