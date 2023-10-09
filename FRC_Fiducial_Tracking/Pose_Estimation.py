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
                tagTvec = cam_pose_to_robot_tvec(self.data_array[i], self.robo_space_pose)

                tvecXCoord = math.sqrt((tagTvec[2]**2) + (tagTvec[0]**2)) * math.cos(self.tag_coords[self.data_array[i].tag_id][4] - self.data_array[i].rvec_y - self.robot_space_pose[5]) - self.robo_space_pose[0]
                tvecYCoord = math.sqrt((tagTvec[2]**2) + (tagTvec[1]**2)) * math.sin(90-self.data_array[i].rvec_x - self.robot_space_pose[3])
                tvecZCoord = math.sqrt((tagTvec[2]**2) + (tagTvec[0]**2)) * math.sin(self.tag_coords[self.data_array[i].tag_id][4] - self.data_array[i].rvec_y - self.robot_space_pose[5]) - self.robo_space_pose[2]

                self.coordinates = [self.data_array[i].tag_id, tvecXCoord, tvecYCoord, tvecZCoord]
                coord_array.append(self.coordinates)

                rvecPitch = -self.data_array[i].rvec_z-self.robot_space_pose[3]
                rvecRoll = -self.data_array[i].rvec_x-self.robot_space_pose[4]
                rvecYaw = self.tag_coords[self.data_array[i].tag_id][4]-self.data_array[i].rvec_y-self.robot_space_pose[5]
        
                self.orientation = [self.data_array[i].tag_id, rvecPitch, rvecRoll, rvecYaw]
                coord_array.append(self.orientation)
        elif(len(self.visible_tags) == 2):
            tag0Tvec = cam_pose_to_robot_tvec(self.data_array[0], self.robo_space_pose)
            tag1Tvec = cam_pose_to_robot_tvec(self.data_array[1], self.robo_space_pose)

            tag0Coords = [self.tag_coords[self.data_array[0].tag_id][0], self.tag_coords[self.data_array[0].tag_id][2], self.tag_coords[self.data_array[0].tag_id][3]]
            tag1Coords = [self.tag_coords[self.data_array[1].tag_id][0], self.tag_coords[self.data_array[1].tag_id][2], self.tag_coords[self.data_array[1].tag_id][3]]
            
            tag_line_dist = distance(tag0Coords[0], tag0Coords[1], tag1Coords[0], tag1Coords[1])
            tag0_cam_dist = math.hypot(tag0Tvec[0], tag0Tvec[2])
            tag1_cam_dist = math.hypot(tag1Tvec[0], tag1Tvec[2])

            cam_angle = math.toDegrees(math.atan(tag0Tvec[0]/tag0Tvec[2])-math.atan(tag1Tvec[0]/tag1Tvec[2]))
            tag0_angle = math.acos(((tag0_cam_dist**2) + (tag_line_dist**2) - (tag1_cam_dist**2)) / (2 * tag0_cam_dist * tag_line_dist))
            tag1_angle = 180-(cam_angle+tag0_angle)

            tvecZCoord = math.hypot(tag0Tvec[0], tag0Tvec[1])

        return coord_array
    
# convert from camera detection translation vectors to robot-relative translation vectors
def cam_pose_to_robot_tvec(tag, robot_space_pose):
    robot_tvec_x = cam_rotation_comp(tag.tvec_x, tag.tvec_z, robot_space_pose[4])[0]
    robot_tvec_y = cam_rotation_comp(tag.tvec_y, tag.tvec_z, robot_space_pose[3])[0]
    robot_tvec_z = cam_rotation_comp(tag.tvec_y, tag.tvec_z, robot_space_pose[3])[1]

    return [robot_tvec_x, robot_tvec_y, robot_tvec_z]

def distance(d1x, d1y, d2x, d2y):
    return math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2)

def slope_angle(x1, x2, y1, y2):
    
    if(x2-x1 != 0):
        return math.toDegrees(math.atan2((y2-y1) / (x2-x1)))
    else:
        return 90.0

# math for converting one individual translation vector into robot-relative translation vector
def cam_rotation_comp(opposite, adjacent, tilt_theta):
    opposite_out = math.toDegrees(math.sin(tilt_theta - math.atan(opposite/adjacent)) * math.hypot(opposite, adjacent))
    adjacent_out = math.toDegrees(math.cos(tilt_theta - math.atan(opposite/adjacent)) * math.hypot(opposite, adjacent))
    return [opposite_out, adjacent_out]

