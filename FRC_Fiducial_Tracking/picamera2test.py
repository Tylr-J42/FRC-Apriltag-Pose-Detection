from picamera2 import Picamera2
from libcamera import Transform
from threading import Thread

# class for allocating a thread to only updating the camera stream,
# the other thread is used for detection processing
class PiVid:

    def __init__(self, camera_res):
        # RPi camera recording setup with threading crap.
        #  For specs - https://www.raspberrypi.com/documentation/accessories/camera.html
        self.camera = Picamera2()
        self.resolution = camera_res
        config = self.camera.create_video_configuration(main={"size": self.resolution}, transform=Transform(hflip=True), lores={"size": (640,480)}, encode='main')
        self.camera.configure(config)
        self.camera.controls({"FrameRate": 120})
        self.frame = None
        self.stopped = False

    # Start camera thread
    def start(self):
        self.camera.start()
        
        Thread(target=self.update, args=()).start()
        return self

    # update camera stream threading
    def update(self):
        self.frame=self.camera.capture_array()
        if self.stopped:
            self.camera.stop()
            return

    # output the frame we want
    def read(self):
        return self.frame

    # end threading
    def stop(self):
        self.stopped = True