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
    
    def rot_params_rv(self, rvecs):
        from math import pi,atan2,asin
        R = cv2.Rodrigues(rvecs)[0]
        roll = 180*atan2(-R[2][1], R[2][2])/pi
        pitch = 180*asin(R[2][0])/pi
        yaw = 180*atan2(-R[1][0], R[0][0])/pi
        rot_params= [roll,pitch,yaw]
        return rot_params

    def cam_to_field(self, tvecs, euler_angles):
        position = [0, 0, 0]
        rotation = [0, 0, 0]

        total_dist = math.sqrt(tvecs[0]**2 + tvecs[1]**2)
        rotation

        return position, rotation

    def calculate_coords(self, image_corners, tags_detected):
        tag_corners = self.tag_corners
        PNP_obj_input = []

        if len(tags_detected)>0:

            for i in range(len(tags_detected)):
                if(len(PNP_obj_input) == 0):
                    PNP_obj_input = [tag_corners[tags_detected[i]][1], tag_corners[tags_detected[i]][2], tag_corners[tags_detected[i]][3], tag_corners[tags_detected[i]][4]]
                else: 
                    PNP_obj_input = PNP_obj_input + [tag_corners[tags_detected[i]][1], tag_corners[tags_detected[i]][2], tag_corners[tags_detected[i]][3], tag_corners[tags_detected[i]][4]]
            PNP_obj_input = np.array(PNP_obj_input)

            #print("PNP_obj_input: ", PNP_obj_input, ", image_corners: ", image_corners, "tags_detected: ", tags_detected)
            ret, rvecs, tvecs = cv2.solvePnP(PNP_obj_input, image_corners, self.camera_matrix, self.dist, flags=0)

            for i in range(len(rvecs)):
                rvecs[i] = math.degrees(rvecs[i])

            euler_angles = self.rot_params_rv(rvecs)
            total_dist = math.hypot(math.hypot(tvecs[0], tvecs[1]), tvecs[2])

            print("euler XYZ: ", euler_angles, "tvecs: ", tvecs, "total_dist: ", total_dist)

            world_angleXYZ = [-euler_angles[0], -euler_angles[1], -euler_angles[2]]
            z_line_offset_coord = []