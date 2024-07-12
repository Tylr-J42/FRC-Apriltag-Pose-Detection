from picamera2 import Picamera2
from libcamera import Transform
import cv2
import time

picam = Picamera2(camera_num=0)
config = picam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"},
                                        transform=Transform(hflip=True),
                                        lores={"size": (640,480)},
                                        encode='main',
                                        )
picam.configure(config)
picam.set_controls({"FrameRate": 120})
picam.start()
while True:
    start_frame = time.time()
    cv2.imshow("frame", picam.capture_array())
    cv2.waitKey(1)
    frame = 1/(time.time()-start_frame)
    print(frame)

