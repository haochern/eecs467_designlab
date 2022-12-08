import math
import iou
import numpy as np
import random
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
    return np.concatenate((matrix[3, :3], transformation.quaternion_from_matrix(matrix[:3, :3])))

def pair_to_edge(prior, posterior):
    homo_pri = transformation.quaternion_matrix(prior[3:7])
    homo_pos = transformation.quaternion_matrix(posterior[3:7])
    homo_pri[:3, 3], homo_pos[:3, 3] = prior[:3], posterior[:3]
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

def homography(point1, point2):
    assert len(point1) == len(point2)
    A = np.zeros((3 * len(point1), 9))
    for i in range(len(point1)):
        x, y = point1[i]
        x_, y_ = point2[i]
        A[2*i, :] = np.array([0, 0, 0, -x, -y, -1, x*y_, y*y_, y_])
        A[2*i + 1, :] = np.array([x, y, 1, 0, 0, 0, -x*x_, -y*x_, -x_])

    U, S, V = np.linalg.svd(A)
    #print(S)
    v_ = V[-1, :].reshape((3, 3))
    H = v_ / v_[2, 2]
    return H

# from https://medium.com/machine-learning-world/linear-algebra-points-matching-with-svd-in-3d-space-2553173e8fed
def euclidean_transform_3D(A, B):
    '''
        A,B - Nx3 matrix
        return:
            R - 3x3 rotation matrix
            t = 3x1 column vector
    '''
    assert len(A) == len(B)

    # number of points
    N = A.shape[0]; 

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre matrices
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # covariance of datasets
    H = np.transpose(AA) * BB

    # matrix decomposition on rotation, scaling and rotation matrices
    U, S, Vt = np.linalg.svd(H)

    # resulting rotation
    R = Vt.T * U.T
    print('R',R)
    #prinyt(Vt)
    print(Vt)
    # handle svd sign problem
    if np.linalg.det(R) < 0:
        print ("sign")
        # thanks to @valeriy.krygin to pointing me on a bug here
        Vt[2,:] *= -1
        R = Vt.T * U.T
        print('new R',R)

    t = -R*centroid_A.T + centroid_B.T

    return R, t

# 3D rigid transform https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t

def ransac(target, current):
    # item.position --> [x,y,z]
    # item.label --> tag
    # target:= item[N],  current:= item[M]
    num_iterations = 1000
    dist_thresh = 2
    best_inliers = np.zeros((1, len(target)))
    best_R = None
    best_t = None

    for i in range(num_iterations):
        indx = random.sample(list(np.arange(len(current))), 5)
        x1_, x2_ = np.array(target)[indx, :], np.array(current)[indx, :]

        R, t = rigid_transform_3D(x1_, x2_)
        new_target = np.matmul(R, current) + t
        inliers = np.sum((new_target.transpose() - target) ** 2, 1) <= dist_thresh
        # print(np.sum((new_x1.transpose() - x2) ** 2, 1))
        #print(np.sum(inliers))
        if np.sum(inliers) > np.sum(best_inliers):
            best_inliers = inliers
            best_R = R
            best_t = t
    return best_R, best_t
    pass