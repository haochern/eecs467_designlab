import math
import numpy as np
import utils



class BoundingBox:
    def __init__(self, corners, tag) -> None:
        self.corners = corners
        self.tag = tag
        pass

    ## These corners should be in global frame

    def volume(self):
        return np.linalg.norm(self.corners[0]-corners[4])*np.linalg.norm(self.corners[0]-self.corners[1])*np.linalg.norm(self.corners[0]-self.corners[2])

    def radius(self):
        return (self.volume()/(math.pi*4/3))**(1/3)
    

    def center(self):
        return [(self.corners[0][0] + self.corners[6][0])/2, (self.corners[0][1] + self.corners[6][1])/2, (self.corners[0][2] + self.corners[6][2])/2] 
        