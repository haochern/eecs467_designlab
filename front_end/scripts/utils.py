import math
import numpy as np
import transformation

def overlap(bbox1, bbox2):
    '''
    Given two bounding box object, find the estimated overlap percentage
    '''



    pass

def adjacent_edge(prior, posterior):
    return posterior @ np.linalg.inv(prior)

def transform_pcd(pcd, pose):
    '''
    pcd: an array of points representing a point cloud in camera's frame [n x 3]
    pose: the camera's position and orientation in global frame
    return the pcd with the orientation transformed into the camera's frame (not the position)
    '''
    camera_transform = transformation.quaternion_matrix(pose.orientation)
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
    # return math.sqrt( (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2 )
    return np.linalg.norm(point1 - point2)