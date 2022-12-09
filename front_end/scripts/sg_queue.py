import math
import numpy as np
from geometry_msgs.msg import Point
from sg_pr.msg import SemanticGraph
from utils import *

MIN_OBJS = 10
SPATIAL_DIS = 5

class SG_queue:
    def __init__(self) -> None:
        self.sg_q = [] # list of bounding boxes in the global frame

        pass

    def insert(self, new_queue):
        print("new q size: ", len(new_queue))
        temp = new_queue.copy()

        for existing_item in self.sg_q:
            different = True
            for item in new_queue:
                print("tags: ", item.tag, existing_item.tag)
                if item.tag == existing_item.tag and overlap(item, existing_item) > 0.5:
                    print("IOU::::::" ,overlap(item, existing_item))
                    different = False
            if different:
                temp.append(existing_item)
        
        self.sg_q = temp
        """
        while (len(new_queue) != 0):
            item = new_queue.pop(0)
            
            for existing_item in self.sg_q:
                print("item.tag = ", item.tag, " existing_item.tag = ", existing_item.tag)
                if (item.tag == existing_item.tag):
                    if (overlap(item, existing_item) > 0.5):
                        self.sg_q.remove(existing_item)
            
            self.sg_q.append(item)
        """
        pass

    def update(self, curr_pose):
        i = len(self.sg_q) - 1
        while (i >= 0):
            if (distance(curr_pose, self.sg_q[i].associated_pose[0:3]) > SPATIAL_DIS):
                self.sq_q.pop(i)
            i -= 1

        pass

    def getGraph(self):
        centers = []
        labels = []
        for bbox in self.sg_q:
            c = bbox.local_center()
            p = Point(x = c[0], y = c[1], z = c[2])
            centers.append(p)
            labels.append(bbox.tag)
        
        semanticGraph = SemanticGraph(centers = centers, nodes = labels)

        return semanticGraph

    def getSize(self):
        return len(self.sg)