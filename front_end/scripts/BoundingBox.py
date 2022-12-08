import math
import numpy as np
import utils



class BoundingBox:
    def __init__(self, corners, tag, pose) -> None:
        self.corners = corners
        self.tag = tag
        self.associated_pose = pose
        pass

    ## These corners should be in global frame

    def volume(self):
        return np.linalg.norm(self.corners[0]-self.corners[4])*np.linalg.norm(self.corners[0]-self.corners[1])*np.linalg.norm(self.corners[0]-self.corners[2])

    def radius(self):
        return (self.volume()/(math.pi*4/3))**(1/3)
    

    def local_center(self):
        return [(self.corners[0][0] + self.corners[6][0])/2, (self.corners[0][1] + self.corners[6][1])/2, (self.corners[0][2] + self.corners[6][2])/2] 
        
    def global_center(self):
        return utils.translate_point(self.local_center(), self.associated_pose)

    def cornersInGlobalFrame(self):
        new_box = utils.transform_bbox(self, self.associated_pose)
        return new_box.corners