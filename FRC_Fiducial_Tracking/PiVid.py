from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread

# class for allocating a thread to only updating the camera stream,
# the other thread is used for detection processing
class PiVid:

    def __init__(self, camera_res):
        # RPi camera recording setup with threading crap.
        #  For specs - https://www.raspberrypi.com/documentation/accessories/camera.html
        self.camera = PiCamera()
        self.camera.resolution = camera_res
        self.camera.framerate = 60
        self.rawCapture = PiRGBArray(self.camera, size=camera_res)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        self.frame = None
        self.stopped = False

    # Start camera thread
    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    # update camera stream threading
    def update(self):
        for frame in self.stream:
            self.frame=frame.array
            self.rawCapture.truncate(0)
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    # output the frame we want
    def read(self):
        return self.frame

    # end threading
    def stop(self):
        self.stopped = True