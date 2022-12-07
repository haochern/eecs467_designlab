import math
import iou
import numpy as np
import transformation

def overlap(bbox1, bbox2):
    '''
    Given two bounding box object, find the estimated overlap percentage
    '''
    bbox1R=bbox1.radius()
    bbox2R=bbox2.radius()
    d = np.linag.norm(bbox2.global_center()-bbox1.global_center())
    if (d > (bbox1R+bbox2R)):
        return 0
    # formula from https://archive.lib.msu.edu/crcmath/math/math/s/s563.htm
    overlap_V= math.pi*(bbox1R+bbox2R-d)**2 * (d**2+2*d*bbox1R+2*d*bbox2R+6*bbox1R*bbox2R-3*bbox1R**2-3*bbox2R**2)/12/d
    voluebbox1= bbox1.volume()
    voluebbox2= bbox2.volume()
    iou= overlap_V/(voluebbox1+voluebbox2-overlap)
    return iou


def matrix_to_tf(matrix):
    return np.r_[matrix[3, :3] + transformation.quaternion_from_matrix(matrix[:3, :3])]

def pair_to_edge(prior, posterior):
    rot_pri = transformation.quaternion_matrix(prior[3, 7])
    rot_pos = transformation.quaternion_matrix(posterior[3, 7])
    homo_pri = np.r_[np.c_[rot_pri, prior[0:3]], [[0, 0, 0, 1]]]
    homo_pos = np.r_[np.c_[rot_pos, posterior[0:3]], [[0, 0, 0, 1]]]
    tf = homo_pos @ np.linalg.inv(homo_pri)
    return matrix_to_tf(tf)

def translate_point(point, position):
    camera_transform = transformation.quaternion_matrix(position)
    point_transform = transformation.translation_matrix(point)
    transformed_point_matrix = np.matmul(camera_transform, point_transform)
    new_point = [transformed_point_matrix[0][3], transformed_point_matrix[1][3], transformed_point_matrix[2][3]]
    return new_point

def transform_pcd(pcd, orientation):
    '''
    pcd: an array of points representing a point cloud in camera's frame [n x 3]
    pose: the camera's position and orientation in global frame
    return the pcd with the orientation transformed into the camera's frame (not the position)
    '''
    camera_transform = transformation.quaternion_matrix(orientation)
    new_pcd = []
    for point in pcd:
        point_transform = transformation.translation_matrix(point)
        transformed_point_matrix = np.matmul(camera_transform, point_transform)
        new_point = [transformed_point_matrix[0][3], transformed_point_matrix[1][3], transformed_point_matrix[2][3]]
        new_pcd.append(new_point)

    return new_pcd

def transform_bbox(bbox, pose):
    '''
    bbox: an object with an array of 8 corners (8 points) and a label in camera's frame
    pose: the camera's position and orientation in global frame
    return the bounding box with corners transformed into the global frame
    '''
    camera_transform = transformation.quaternion_matrix(pose.position)
    new_corners = []
    for corner in bbox.corners:
        point_transform = transformation.translation_matrix(corner)
        transformed_corner_matrix = np.matmul(camera_transform, point_transform)
        new_corner = [transformed_corner_matrix[0][3], transformed_corner_matrix[1][3], transformed_corner_matrix[2][3]]
        new_corners.append(new_corner)
    bbox.corners = new_corners

    return bbox

def distance(point1, point2):
    '''
    point1, point2: array of size 3 points representing a point [x,y,z]
    '''
    return np.linalg.norm(point1 - point2)

def random_sampling(pc, num_sample, replace=None, return_choices=False):
    """ Input is NxC, output is num_samplexC
    """
    '''
    Copy from pc_util,py sampling pcd point cloud
    '''
    if replace is None: replace = (pc.shape[0]<num_sample)
    choices = np.random.choice(pc.shape[0], num_sample, replace=replace)
    if return_choices:
        return pc[choices], choices
    else:
        return pc[choices]