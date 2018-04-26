#!/usr/bin/python
from __future__ import division
import numpy as np


def ls_line_intersection3d(start, end):
    '''
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    http://www.mathworks.com/matlabcentral/fileexchange/37192-intersection-point-of-lines-in-3d-space
    Note: Not performing very well as of yet
    '''
    if len(start) != len(end):
        raise RuntimeError('Dimension mismatch')
    if len(start) < 2:
        raise RuntimeError('Insufficient line count ({})'.format(len(start)))
    dir_vecs = end - start
    lengths = np.linalg.norm(dir_vecs).reshape((-1, 1))
    norm_dir_vecs = dir_vecs / lengths
    nx = norm_dir_vecs[:, 0]
    ny = norm_dir_vecs[:, 1]
    nz = norm_dir_vecs[:, 2]
    XX = nx * nx - 1
    YY = ny * ny - 1
    ZZ = nz * nz - 1
    XY = nx * ny
    XZ = nx * nz
    YZ = ny * nz
    S = np.array([[np.sum(XX), np.sum(XY), np.sum(XZ)],
                  [np.sum(XY), np.sum(YY), np.sum(YZ)],
                  [np.sum(XZ), np.sum(YZ), np.sum(ZZ)]])
    AX = start[:, 0]
    AY = start[:, 1]
    AZ = start[:, 2]
    CX = np.sum(AX * XX + AY * XY + AZ * XZ)
    CY = np.sum(AX * XY + AY * YY + AZ * YZ)
    CZ = np.sum(AX * XZ + AY * YZ + AZ * ZZ)
    C = np.stack((CX, CY, CZ))
    return np.linalg.pinv(S).dot(C)


def ls_line_intersection3d_(start, end):
    '''
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf
    Note: Not working correctly yet
    '''
    dir_vecs = end - start
    lengths = np.linalg.norm(dir_vecs).reshape((-1, 1))
    dir_vecs = dir_vecs / lengths
    I = np.eye(3)
    R = np.zeros((3, 3))
    Q = np.zeros(3)
    for i, N in enumerate(dir_vecs):
        r = I - N.T.dot(N)
        R += r
        q = r.dot(start[i])
        Q += q
    return np.linalg.pinv(R).dot(Q)


def ls_line_intersection2d(start, end):
    """
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    https://en.wikipedia.org/wiki/Line-line_intersection#In_two_dimensions_2
    """
    if len(start) != len(end):
        raise RuntimeError('Dimension mismatch')
    if len(start) < 2:
        raise RuntimeError('Insufficient line count ({})'.format(len(start)))
    dir_vecs = end - start
    lengths = np.linalg.norm(dir_vecs).reshape((-1, 1))
    dir_vecs = dir_vecs / lengths
    Rl_90 = np.array([[0, -1], [1, 0]])  # rotates right 90deg
    perp_unit_vecs = Rl_90.dot(dir_vecs.T).T
    A_sum = np.zeros((2, 2))
    Ap_sum = np.zeros((2, 1))

    for x, y in zip(start, perp_unit_vecs):
        A = y.reshape(2, 1).dot(y.reshape(1, 2))
        Ap = A.dot(x.reshape(2, 1))
        A_sum += A
        Ap_sum += Ap

    return np.linalg.lstsq(A_sum, Ap_sum)[0]
