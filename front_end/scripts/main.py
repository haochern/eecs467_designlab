#!/usr/bin/env python
import numpy as np
import time

import rospy
import ros_numpy

from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from sg_pr.msg import EvalPackage, EvalScore

from votenet.msg import BoundingBox, BBoxArray, PointCloud

from utils import *

from sg_queue import *
from factor_graph import *
from BoundingBox import *

from transformation import *

THRESHOLD = 0.8
DELAY = 30
NUM_POINT = 8000

TRANSFORM_CAMERA = quaternion_from_euler(math.pi/2,-math.pi/2,0)

UPDATE_DISTANCE = 1
MIN_OBJC_COUNT = 3

FG = None

class FrontEnd:
    
    def __init__(self) -> None:
        self.sg_q = SG_queue()
        self.fg = FactorGraph()
        self.all_graphs = []

        self.timestamp = 0
        self.receipt = 0
        self.pendingPcd = {}
        self.pendingPose = {}

        FG = self.fg

        self.subscriber_pose = rospy.Subscriber('orb_slam2_rgbd/pose', PoseStamped, self.pose_callback)
        self.subscriber_pcd = rospy.Subscriber('points2', PointCloud2, self.pcd_callback)
        self.subscriber_bboxes = rospy.Subscriber('bbox', BBoxArray, self.bboxes_callback)
        self.subscriber_score = rospy.Subscriber('score', EvalScore, self.eval_callback)
        self.publisher_pcd = rospy.Publisher('pcd_msg', PointCloud, queue_size=10)
        self.publisher_pr = rospy.Publisher('sg', EvalPackage, queue_size=10)


        self.last_published_pose = None

        self.t0 = time.time()

    def pose_callback(self, msg: PoseStamped):
        self.receipt += 1
        if self.receipt % DELAY == 0:
            if len(self.pendingPcd) > 0:
                print("Point Cloud(ID: ", self.receipt, ") pop")  
                pcd = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pendingPcd.pop(self.receipt, None), remove_nans=True)
                # build up the factor graph
                pos, ori = msg.pose.position, msg.pose.orientation
                curr_camera_pose = [pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z]
                vtx, edge = self.fg.add_adjacent_vertex(curr_camera_pose)
                print("new vertex:", vtx)
                print("new edge:", edge)
                print("curr camera pose: ", curr_camera_pose)

                # publish to votenet for detection
                downSampled_pcd = random_sampling(pcd,NUM_POINT)  
                pcd = transform_pcd(downSampled_pcd, TRANSFORM_CAMERA)
                pcd = transform_pcd(pcd, curr_camera_pose[3:7]) # comment this line if not use down sampling
                # pcd = transform_pcd(pcd, curr_camera_pose[3:7]) # uncomment this line if not use down sampling
                pcd_msg = PointCloud(receipt = self.receipt, points = [Float32MultiArray(data=p) for p in pcd])
                self.publisher_pcd.publish(pcd_msg) # to votenet
            else:
                self.pendingPose[self.receipt] = msg
                print("Pose(ID: ", self.receipt, ") pushed")
        

            


    def pcd_callback(self, msg: PointCloud2):  
        # print(self.timestamp)
        self.timestamp += 1
        if self.timestamp % DELAY == 0:
            t0 = time.time()
            if len(self.pendingPose) > 0:
                print("Pose(ID: ", self.timestamp, ") pop")  
                pcd = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
                # build up the factor graph
                poped_msg = self.pendingPose.pop(self.timestamp, None)
                pos, ori = poped_msg.pose.position, poped_msg.pose.orientation
                curr_camera_pose = [pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z]
                vtx, edge = self.fg.add_adjacent_vertex(curr_camera_pose)
                print("new vertex:", vtx)
                print("new edge:", edge)
                print("curr camera pose: ", curr_camera_pose)

                # publish to votenet for detection
                downSampled_pcd = random_sampling(pcd,NUM_POINT)  
                pcd = transform_pcd(downSampled_pcd, TRANSFORM_CAMERA)
                pcd = transform_pcd(pcd, curr_camera_pose[3:7]) # comment this line if not use down sampling
                # pcd = transform_pcd(pcd, curr_camera_pose[3:7]) # uncomment this line if not use down sampling
                pcd_msg = PointCloud(receipt = self.timestamp, points = [Float32MultiArray(data=p) for p in pcd])
                self.publisher_pcd.publish(pcd_msg) # to votenet
            else:
                self.pendingPcd[self.timestamp] = msg
                print("Point Cloud(ID: ", self.timestamp, ") pushed")
        
            # print("-----------Time passed: ", time.time() - t0)

    def bboxes_callback(self, msg: BBoxArray): 
        t0 = time.time()
        actual_bbox = []
        print("message receipt", msg.receipt)
        # print(len(self.fg.vertexes))
        # print((int)(msg.receipt/DELAY))
        associated_pose = self.fg.vertexes[(int)(msg.receipt/DELAY)-1]
        print("length of msg array: ", len(msg.array))
        for bbox in msg.array:
            if bbox.score < 0.9:
                continue
            corners = []
            for p in bbox.bbox_corners:
                corners.append([p.x, p.y, p.z])
            actual_bbox.append(BoundingBox(corners=corners, tag=bbox.tag, pose=associated_pose))
        
        self.sg_q.insert(new_queue=actual_bbox)

        print("Current SQ_Q Size: ",len(self.sg_q.getGraph().nodes))
        print("Current All graph Size: ",len(self.all_graphs))
        # to SG_PR
        if self.last_published_pose != None:
            print("Update Distance", utils.distance(self.last_published_pose[0:3], associated_pose[0:3]))
        if self.last_published_pose == None and self.sg_q.getSize() > MIN_OBJC_COUNT:
            self.all_graphs.append(self.sg_q.getGraph())
            self.last_published_pose = associated_pose
            print("Update!!!")
        elif self.last_published_pose != None and utils.distance(self.last_published_pose[0:3], associated_pose[0:3]) > UPDATE_DISTANCE and self.sg_q.getSize() != 0:
            self.sg_q.update(curr_pose=getVectorForm(associated_pose[0:3]))
            targetGraph = self.sg_q.getGraph() 
            evalPackage = EvalPackage(receipt=msg.receipt, batch=self.all_graphs, target=targetGraph) 
            self.last_published_pose = associated_pose
            self.t0 = time.time()
            self.publisher_pr.publish(evalPackage)
            print("Update!!!")
            # publish
        # print("-------------------------------")
        # print("Time Passed: ", time.time() - t0)


    def eval_callback(self, msg: EvalScore):
        
        score, receipt = msg.score, msg.receipt
        print("Eval Call Back..............score................", score)
        print("-----------------Time passed : ", time.time() - self.t0)
        idx = np.argmax(score)
        best_score = score[idx]
        if best_score > THRESHOLD:
            # loop closure
            self.fg.add_edge(None, (int)(receipt/DELAY), pair_to_edge(None, self.fg.vertexes[(int)(receipt/30)]))
            pass
        else:
            # all graphs append
            self.all_graphs.append(self.sg_q.getGraph())
            pass

def main():
    rospy.init_node("front-end")

    f = FrontEnd()

    rospy.spin()


if __name__ == '__main__':
    main()