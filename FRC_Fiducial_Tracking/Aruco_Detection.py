import cv2
import numpy as np
from Picam2Vid import Picam2Vid
import time


stream = Picam2Vid((1536, 864))

tag36h11 = cv2.aruco.DICT_APRILTAG_36h11
params = cv2.aruco.DetectorParameters_create()
dictionary = cv2.aruco.Dictionary_get(tag36h11)

while True:
    start=time.time()

    stream.update()
    frame = stream.read()

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('frame', frame)
    cv2.waitKey(1)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(grey, dictionary, parameters=params)
    #print(corners)
    print("ids: " + str(ids))
    #print(rejected)

    print("FPS: "+str(1/(time.time()-start)))
