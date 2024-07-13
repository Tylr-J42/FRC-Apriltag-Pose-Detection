from Picam2Vid import Picam2Vid
import cv2

camera = Picam2Vid((640, 480))

while True:
    frame = camera.read()

    cv2.imshow("worknow", frame)
    cv2.waitKey(1)