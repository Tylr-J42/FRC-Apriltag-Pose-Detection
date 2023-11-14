import math
import cv2
import numpy as np

class PNPPose:

    def __init__(self, tag_corners, robo_space_pose, camera_matrix, dist):
        self.tag_corners = tag_corners
        self.dist = dist
        self.camera_matrix = camera_matrix
        self.orientation = [0, 0, 0]
        self.robo_space_pose = robo_space_pose
        
    def calculate_coords(self, image_corners, tags_detected):
        tag_corners = self.tag_corners
        PNP_obj_input = np.array([])

        for i in range(len(tags_detected)):
            if(PNP_obj_input.size == 0):
                PNP_obj_input = np.array([tag_corners[tags_detected[i]][1], tag_corners[tags_detected[i]][2], tag_corners[tags_detected[i]][3], tag_corners[tags_detected[i]][4]])
            else: 
                PNP_obj_input = np.stack([PNP_obj_input, np.array([tag_corners[tags_detected[i]][1], tag_corners[tags_detected[i]][2], tag_corners[tags_detected[i]][3], tag_corners[tags_detected[i]][4]])])

        print("PNP_obj_input: ", PNP_obj_input, ", image_corners: ", image_corners)
        ret, rvecs, tvecs = cv2.solvePnP(PNP_obj_input, image_corners, self.camera_matrix, self.dist, flags=0)
        print(tvecs,"poop")
