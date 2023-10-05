import math
from TagObj import TagObj

class PoseEstimation:

    def __init__(self, tag_coords, robo_space_pose):
        self.tag_coords = tag_coords
        self.robot_space_pose = robo_space_pose
        self.orientation = [0, 0, 0, 0]
        self.coordinates = [0, 0, 0, 0]

    def update(self, data_array):
        self.visible_tags = len(data_array)

    def getCoords(self):
        coord_array = []
        if(len(self.visible_tags)>1):
            for i in self.visible_tags:
                tvecXCoord = math.sqrt(self.data_array[i].tvec_z**2 + self.data_array[i].tvec_x**2) * math.cos(self.tag_coords[self.data_array[i].tag_id][4] - self.data_array[i].rvec_y - self.robot_space_pose[5]) - self.robo_space_pose[0]
                tvecYCoord = math.sqrt(self.data_array[i].tvec_z**2 + self.data_array[i].tvec_y**2) * math.sin(90-self.data_array[i].rvec_x - self.robot_space_pose[3])
                tvecZCoord = math.sqrt(self.data_array[i].tvec_z**2 + self.data_array[i].tvec_x**2) * math.sin(self.tag_coords[self.data_array[i].tag_id][4] - self.data_array[i].rvec_y - self.robot_space_pose[5]) - self.robo_space_pose[2]

                self.coordinates = [self.data_array[i].tag_id, tvecXCoord, tvecYCoord, tvecZCoord]
                coord_array.append(self.coordinates)

                rvecPitch = -self.data_array[i].rvec_z-self.robot_space_pose[3]
                rvecRoll = -self.data_array[i].rvec_x-self.robot_space_pose[4]
                rvecYaw = self.tag_coords[self.data_array[i].tag_id][4]-self.data_array[i].rvec_y-self.robot_space_pose[5]
        
                self.orientation = [self.data_array[i].tag_id, rvecPitch, rvecRoll, rvecYaw]
                coord_array.append(self.orientation)
        

        return coord_array
