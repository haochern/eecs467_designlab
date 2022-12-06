import math
import numpy as np
import transformation

def overlap(bbox1, bbox2):
    pass

def transform_pcd(pcd, pose):
    '''
    pcd: an array of points representing a point cloud in camera's frame
    pose: the camera's position and orientation in global frame
    return the pcd with the orientation transformed into the camera's frame (not the position)
    '''
    pass

def transform_bbox(bbox, pose):
    '''
    bbox: an object with an array of 8 corners (8 points) and a label in camera's frame
    pose: the camera's position and orientation in global frame
    return the bounding box with corners transformed into the global frame
    '''
    camera_transform = transformation.quaternion_matrix(pose.orientation)
    for corner in bbox.corners:
        point_transform = transformation.translation_matrix(corner)
        transformed_corner_matrix = np.matmul(camera_transform, point_transform)
        corner = [transformed_corner_matrix[0][3], transformed_corner_matrix[1][3], transformed_corner_matrix[2][3]]

    return bbox

def distance(point1, point2):
    '''
    point1, point2: array of size 3 points representing a point [x,y,z]
    '''
    # return math.sqrt( (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2 )
    return np.linalg.norm(point1 - point2)