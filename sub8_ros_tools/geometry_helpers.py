from __future__ import division
import numpy as np


def make_rotation(vector_a, vector_b):
    '''Determine a 3D rotation that rotates A onto B
        In other words, we want a matrix R that aligns a with b

        >> R = make_rotation(a, b)
        >> p = R.dot(a)
        >> np.dot(p, a) 
        >>>  0.0

        [1] Calculate Rotation Matrix to align Vector A to Vector B in 3d?
            http://math.stackexchange.com/questions/180418
        [2] N. Ho, Finding Optimal Rotation...Between Corresponding 3D Points
            http://nghiaho.com/?page_id=671
    '''
    unit_a = normalize(vector_a)
    unit_b = normalize(vector_b)

    v = np.cross(unit_a, unit_b)
    s = np.linalg.norm(v)

    c = np.dot(unit_a, unit_b)

    skew_cross = skew_symmetric_cross(v)
    skew_squared = np.linalg.matrix_power(skew_cross, 2)

    if np.isclose(c, 1.0, atol=1e-4):
        R = np.eye(3)
        return R
    elif np.isclose(c, -1.0, atol=1e-4):
        R = np.eye(3)
        R[2, 2] *= -1
        return R

    normalization = (1 - c) / (s ** 2)

    R = np.eye(3) + skew_cross + (skew_squared * normalization)

    # Address the reflection case
    if np.linalg.det(R) < 0:
        R[:, 3] *= -1

    return R

def skew_symmetric_cross(a):
    '''Return the skew symmetric matrix representation of a vector
        [1] https://en.wikipedia.org/wiki/Cross_product#Skew-symmetric_matrix
    '''
    assert len(a) == 3, "a must be in R3"
    skew_symm = np.array([
        [0.,     -a[2], +a[1]],
        [+a[2],     0., -a[0]],
        [-a[1],  +a[0],    0.],
    ], dtype=np.float32)
    return skew_symm

def normalize(vector):
    return vector / np.linalg.norm(vector)