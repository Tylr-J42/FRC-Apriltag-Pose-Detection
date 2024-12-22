import cv2
import time

stream = cv2.VideoCapture(0)

stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

print(stream.get(cv2.CAP_PROP_EXPOSURE))

stream.set(cv2.CAP_PROP_FPS, 100.0)
stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
stream.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual mode
stream.set(cv2.CAP_PROP_EXPOSURE, 10)
time.sleep(2)
print(stream.get(cv2.CAP_PROP_EXPOSURE))


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

cv2.destroyAllWindows()
stream.stop()