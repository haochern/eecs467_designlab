#!/home/hao/anaconda3/envs/votenet/bin/python
import os
import numpy as np
import importlib
import time

import torch
import torch.optim as optim

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point

from votenet.msg import BoundingBox, BBoxArray, PointCloud

from votenet.utils.pc_util import random_sampling
from votenet.models.ap_helper import parse_predictions

from votenet.sunrgbd.sunrgbd_detection_dataset import DC

from votenet.models.ap_helper import flip_axis_to_depth


BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'src/votenet'))

# NUM_POINT = 40000


class VoteNet_ros:
    def __init__(self, net, device, eval_config_dict) -> None:
        self.net = net
        self.device = device
        self.eval_config_dict = eval_config_dict

        self.publisher_ = rospy.Publisher('bbox', BBoxArray, queue_size=10)
        self.subscriber_ = rospy.Subscriber('cloud', PointCloud, self.votenet_callback)


    def votenet_callback(self, msg: PointCloud):

        # pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        
        pc = [p.position for p in msg.points]
        
        detections = self.votenet_evaluation(pc)

        msg = BBoxArray()
        for obj in detections:
            bbox = BoundingBox()

            bbox.tag = DC.class2type[obj[0]]
            bbox.score = obj[2]
            bbox.bbox_corners = [Point(coord[0], coord[1], coord[2]) for coord in flip_axis_to_depth(obj[1])]
            msg.array.append(bbox)
        self.publisher_.publish(msg)


    def preprocess_point_cloud(self, point_cloud):
        ''' Prepare the numpy point cloud (N,3) for forward pass '''
        point_cloud = point_cloud[:,0:3] # do not use color for now
        floor_height = np.percentile(point_cloud[:,2],0.99)
        height = point_cloud[:,2] - floor_height
        point_cloud = np.concatenate([point_cloud, np.expand_dims(height, 1)],1) # (N,4) or (N,7)
        # point_cloud = random_sampling(point_cloud, NUM_POINT)
        pc = np.expand_dims(point_cloud.astype(np.float32), 0) # (1,40000,4)
        return pc


    def votenet_evaluation(self, point_cloud):
        pc = self.preprocess_point_cloud(point_cloud)
        print('Loaded point cloud data: ROS_TOPIC')
    
        # Model inference
        inputs = {'point_clouds': torch.from_numpy(pc).to(self.device)}
        tic = time.time()
        with torch.no_grad():
            end_points = self.net(inputs)
        toc = time.time()
        print('Inference time: %f'%(toc-tic))
        end_points['point_clouds'] = inputs['point_clouds']
        pred_map_cls = parse_predictions(end_points, self.eval_config_dict)
        print('Finished detection. %d object detected.'%(len(pred_map_cls[0])))
        for i in range(len(pred_map_cls[0])):
            print(DC.class2type[pred_map_cls[0][i][0]], pred_map_cls[0][i][2])
        return pred_map_cls[0]


def main():
    rospy.init_node("votenet")
    
    demo_dir = os.path.join(BASE_DIR, 'demo_files') 
    checkpoint_path = os.path.join(demo_dir, 'pretrained_votenet_on_sunrgbd.tar')

    eval_config_dict = {'remove_empty_box': True, 'use_3d_nms': True, 'nms_iou': 0.25,
        'use_old_type_nms': False, 'cls_nms': False, 'per_class_proposal': False,
        'conf_thresh': 0.5, 'dataset_config': DC}

    # Init the model and optimzier
    MODEL = importlib.import_module('votenet.models.votenet') # import network module
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net = MODEL.VoteNet(num_proposal=256, input_feature_dim=1, vote_factor=1,
        sampling='seed_fps', num_class=DC.num_class,
        num_heading_bin=DC.num_heading_bin,
        num_size_cluster=DC.num_size_cluster,
        mean_size_arr=DC.mean_size_arr).to(device)
    print('Constructed model.')


    # Load checkpoint
    optimizer = optim.Adam(net.parameters(), lr=0.001)
    checkpoint = torch.load(checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epoch = checkpoint['epoch']
    print("Loaded checkpoint %s (epoch: %d)"%(checkpoint_path, epoch))
    net.eval() # set model to eval mode (for bn and dp)

    votenet = VoteNet_ros(net, device, eval_config_dict)
    rospy.spin()



if __name__ == '__main__':
    main()