import math
import numpy
import utils

from utils import *

MIN_OBJS = 10
SPATIAL_DIS = 5

class BoundingBox:
    def __init__(self, corners, tag) -> None:
        self.corners = corners
        self.tag = tag
        pass

    def center(self):
        return [(self.corners[0][0] + self.corners[6][0])/2, (corners[0][1] + self.corners[6][1])/2, (self.corners[0][2] + self.corners[6][2])/2] 
        

class SG_queue:
    def __init__(self) -> None:
        self.sg_q = [] # list of bounding boxes in the global frame

        pass

    def insert(self, new_queue):
        while (len(new_queue) != 0):
            item = new_queue.pop(0)
            for existing_item in self.sg_q:
                if (item.label == existing_item.label):
                    if (utils.overlap(item, existing_item) < 0.8):
                        self.sg_q.append(item)
        pass

    def update(self, curr_pose):
        i = len(self.sg_q)
        while (i > 0):
            if (utils.distance(curr_pose.position, self.sg_q[i].center()) > SPATIAL_DIS):
                self.sq_q.pop(i)

        pass