
class TagObj:

    def __init__(self, tag_id, tvecs, rvecs, totalDist):
        self.tag_id = tag_id
        self.tvec_x = tvecs[0]
        self.tvec_y = tvecs[1]
        self.tvec_z = tvecs[2]
        self.rvec_x = rvecs[0]
        self.rvec_y = rvecs[1]
        self.rvec_z = rvecs[2]
        self.totalDist = totalDist

    def getID(self):
        return self.tag_id

    def getTvecX(self):
        return self.tvec_x
    
    def getTvecY(self):
        return self.tvec_y
    
    def getTvecZ(self):
        return self.tvec_z

    def getRvecX(self):
        return self.rvec_x
    
    def getRvecY(self):
        return self.rvec_y
    
    def getRvecZ(self):
        return self.rvec_z
    
    def getTotalDist(self):
        return self.totalDist
        