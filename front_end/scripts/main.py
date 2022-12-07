#!/usr/bin/env python
import numpy as np

import rospy
import ros_numpy

from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from votenet.msg import BoundingBox, BBoxArray

from sg_queue import *
from factor_graph import *
from BoundingBox import *

THRESHOLD = 0.8
DELAY = 30

class FrontEnd:
    
    def __init__(self) -> None:
        self.sg_q = SG_queue()
        self.fg = FactorGraph()

        self.timestamp = 0
        self.receipt = 0
        self.pending = {}


        self.subscriber_pose = rospy.Subscriber('orb_slam2_rgbd/pose', PoseStamped, self.pose_callback)
        self.subscriber_pcd = rospy.Subscriber('Point2', PointCloud2, self.pcd_callback)
        self.subscriber_bboxes = rospy.Subscriber('bbox', BBoxArray, self.bboxes_callback)
        self.subscriber_score = rospy.Subscriber('score', Float32MultiArray, self.eval_callback)
        self.publisher_pcd = rospy.Publisher('pcd_msg', PointCloud2, queue_size=10)

    def pose_callback(self, msg: PoseStamped):
        self.receipt += 1
        if self.receipt % DELAY == 0:
            pcd = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pending.pop(self.receipt, None))

            pos, ori = msg.pose.position, msg.pose.orientation
            curr_camera_pose = [pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z]
            self.fg.add_adjacent_vertex(curr_camera_pose)
            
            # TODO: down sampling
            pcd = transform_pcd(pcd, curr_camera_pose[3:7])
            pcd_msg = ros_numpy.point_cloud2.xyz_array_to_pointcloud2(pcd, msg.header.stamp, 'map')
            self.publisher_pcd.publish(pcd_msg) # to votenet

    def pcd_callback(self, msg: PointCloud2):        
        self.timestamp += 1
        if self.timestamp % DELAY == 0:
            self.pending[self.timestamp] = msg

    def bboxes_callback(self, msg: BBoxArray): 
        actual_bbox = []
        for bbox in msg:
            corners = []
            for p in msg.bbox_corners:
                corners.append([p.x, p.y, p.z])
            actual_bbox.append(BoundingBox(corners=corners, tag=bbox.tag, pose=None)) # TODO: add associated pose
        
        SG_queue.insert(new_queue=actual_bbox)
        SG_queue.update(curr_pose=None) # TODO: get a current pose

    def eval_callback(self, msg: Float32MultiArray):

        pass

def main():
    rospy.init_node("Front-End")

    f = FrontEnd()

    rospy.spin()


if __name__ == '__main__':
    main()