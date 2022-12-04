#!/usr/bin/env python
import numpy as np

import rospy

from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from votenet.msg import BoundingBox, BBoxArray

THRESHOLD = 0.8


class frontend:
    def __init__(self) -> None:
        self.graph_set = []
        self.curr_camera_pose = np.zeros(7)

        self.subscriber_pose = rospy.Subscriber('orb_slam2_rgbd/pose', PoseStamped, self.pose_callback)
        self.subscriber_pcd = rospy.Subscriber('Point2', PointCloud2, self.pcd_callback)
        self.subscriber_bboxes = rospy.Subscriber('bbox', BBoxArray, self.bboxes_callback)
        self.subscriber_score = rospy.Subscriber('score', Float32MultiArray, self.eval_callback)
        self.publisher_ = rospy.Publisher('test', Int32, queue_size=10)

    def pose_callback(self, msg: PoseStamped):
        self.curr_camera_pose = msg.pose.position + msg.pose.orientation

    def pcd_callback(self, msg: PointCloud2):
        self.graph_set.append(msg.data)
        print(self.graph_set)
        pass

    def bboxes_callback(self, msg: Float32MultiArray):
        pass

    def eval_callback(self, msg: Float32MultiArray):
        pass

def main():
    rospy.init_node("Front-End")

    f = frontend()

    rospy.spin()


if __name__ == '__main__':
    main()