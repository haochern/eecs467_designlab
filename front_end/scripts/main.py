#!/usr/bin/env python
import numpy as np

import rospy

from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from votenet.msg import BoundingBox, BBoxArray

from sg_queue import *
from factor_graph import *


THRESHOLD = 0.8
pcd_counter = 0
pose_counter = 0
pcd_to_be_sent_to_votenet = {}


class FrontEnd:
    
    def __init__(self) -> None:
        self.sg_q = SG_queue()
        self.fg = FactorGraph()

        # self.curr_camera_pose = np.zeros(7)

        self.subscriber_pose = rospy.Subscriber('orb_slam2_rgbd/pose', PoseStamped, self.pose_callback)
        self.subscriber_pcd = rospy.Subscriber('Point2', PointCloud2, self.pcd_callback)
        self.subscriber_bboxes = rospy.Subscriber('bbox', BBoxArray, self.bboxes_callback)
        self.subscriber_score = rospy.Subscriber('score', Float32MultiArray, self.eval_callback)
        self.publisher_ = rospy.Publisher('test', Int32, queue_size=10)

    def pose_callback(self, msg: PoseStamped):
        pos, ori = msg.pose.position, msg.pose.orientation
        curr_camera_pose = [pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z]
        self.fg.add_adjacent_vertex(curr_camera_pose)
        
        self.curr_camera_pose = msg.pose.position + msg.pose.orientation
        if pose_counter % 30 == 0:
            # find associated pcd
            # publish to votenet (pcd + pose)
            # delete associated pcd from array
            pass

        pose_counter += 1

    def pcd_callback(self, msg: PointCloud2):
        self.graph_set.append(msg.data)
        print(self.graph_set)

        if pcd_counter % 30 == 0:
            pcd_to_be_sent_to_votenet.append(msg)
            pass

        # send to orbslam
        pcd_counter += 1
        pass

    def bboxes_callback(self, msg: BBoxArray):

        pass

    def eval_callback(self, msg: Float32MultiArray):
        pass

def main():
    rospy.init_node("Front-End")

    f = FrontEnd()

    rospy.spin()


if __name__ == '__main__':
    main()