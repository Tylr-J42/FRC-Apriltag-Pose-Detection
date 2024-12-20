import cv2
import os

stream = cv2.VideoCapture(0)

path = "/home/tyler/Desktop/FRC-Apriltag-Pose-Detection/FRC_Fiducial_Tracking/Static_Tag_Pics/"
    
stream.set(cv2.CAP_PROP_BRIGHTNESS, 64)
stream.set(cv2.CAP_PROP_CONTRAST, 0)

def getFrame():
    ret, output = stream.read()
    if(ret):
        return output
    else:
        getFrame()

while True:
    frame = getFrame()
    cv2.imshow("frame", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key ==ord("q"):
        break
    
'''
cv2.destroyAllWindows()
confirmation = input("keep y or n: ")
print(frame)
if confirmation == "y":
    file_order = len(os.listdir(path))
    cv2.imwrite(path+str(file_order)+".jpg", frame)
stream.stop()
'''