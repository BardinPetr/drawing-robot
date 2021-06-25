import math

import numpy as np
from scipy.spatial.transform import Rotation as R


def rv2rpy(data):
    return R.from_rotvec(data).as_euler('xyz', True)


def rpy2rv(data):
    return R.from_euler('xyz', data, True).as_rotvec()


def rotation_from_vectors(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return R.from_matrix(rotation_matrix)


def translate(mas, rx, ry, rz, ox, oy, oz, width, height, pixel_width, pixel_height, inverse=False):
    r = R.from_euler('xyz', [rx, ry, rz], degrees=True)

    result = [0] * len(mas)

    for i in range(len(mas)):
        result[i] = [0] * len(mas[i])
        for j in range(len(mas[i])):
            mat = np.zeros((3,), np.float64)
            mat[0] = mas[i][j][0] * width / pixel_width * (-1 if inverse else 1)
            mat[1] = mas[i][j][1] * height / pixel_height

            mat = r.apply(mat)

            mat[0] += ox
            mat[1] += oy
            mat[2] += oz

            result[i][j] = np.zeros((3,), np.float64)
            result[i][j][0] = mat[0]
            result[i][j][1] = mat[1]
            result[i][j][2] = mat[2]

    return result


def translate_one(vec, rx, ry, rz):
    return translate([[vec]], rx, ry, rz, 0, 0, 0, 1, 1, 1, 1)[0][0]
