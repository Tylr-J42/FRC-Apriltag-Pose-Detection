import cv2
import numpy as np
from Picam2Vid import Picam2Vid



while True:
    stream = Picam2Vid()
    stream.update()
    frame = cv2.imread(stream.read())
    cv2.imshow('frame', frame)
    cv2.waitKey(0)

    tag36h11 = cv2.aruco.DICT_APRILTAG_36h11
    params = cv2.aruco.DetectorParameters_create()

    (corners, ids, rejcted) = cv2.aruco.detectMarkers(frame, tag36h11, parameters=params)
    print(corners)
    print(ids)

