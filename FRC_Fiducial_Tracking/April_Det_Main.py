import argparse
import ntcore
from April_PNP_Live import PNP_Detection
import time

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
#parser.add_argument("--high_res", action='store_true', help="high resolution camera capture")
parser.add_argument("--pose_estimation", action='store_true', help="estimate pose based on detected tags")
args = parser.parse_args()

# network tables + RoboRio IP
inst = ntcore.NetworkTableInstance.getDefault()
vision_table = inst.getTable("Fiducial")

tag3tx = vision_table.getDoubleTopic("tag3tx").publish()
tag3ty = vision_table.getDoubleTopic("tag3ty").publish()
FPS_topic = vision_table.getDoubleTopic("fps").publish()

inst.startClient4("client")
inst.setServerTeam(2648)

detector1 = PNP_Detection(0)
time.sleep(0.1)

while True:

    tx3, ty3, FPS = detector1.update(args.display)

    tag3tx.set(tx3)
    tag3ty.set(ty3)
    FPS_topic.set(FPS)