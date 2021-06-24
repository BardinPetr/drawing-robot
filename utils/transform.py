import math

import numpy as np
from scipy.spatial.transform import Rotation as R


def rv2rpy(data):
    return R.from_rotvec(data).as_euler('xyz', True)


def rpy2rv(data):
    return R.from_euler('xyz', data, True).as_rotvec()


def rotation_matrix_from_vectors(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def translate(mas, rx, ry, rz, ox, oy, oz, width, height, pixel_width, pixel_height):
    rotX = np.zeros((3, 3), np.float32)
    rotY = np.zeros((3, 3), np.float32)
    rotZ = np.zeros((3, 3), np.float32)

    rotX[0][0] = 1
    rotX[1][1] = math.cos(rx)
    rotX[1][2] = -math.sin(rx)
    rotX[2][1] = math.sin(rx)
    rotX[2][2] = math.cos(rx)

    rotY[0][0] = math.cos(ry)
    rotY[0][2] = math.sin(ry)
    rotY[1][1] = 1
    rotY[2][0] = -math.sin(ry)
    rotY[2][2] = math.cos(ry)

    rotZ[0][0] = math.cos(rz)
    rotZ[0][1] = -math.sin(rz)
    rotZ[1][0] = math.sin(rz)
    rotZ[1][1] = math.cos(rz)
    rotZ[2][2] = 1

    result = [0] * len(mas)

    for i in range(len(mas)):
        result[i] = [0] * len(mas[i])
        for j in range(len(mas[i])):
            mat = np.zeros((3, 1), np.float64)
            mat[0][0] = mas[i][j][0] * width / pixel_width
            mat[1][0] = mas[i][j][1] * height / pixel_height

            mat = np.dot(rotX, mat)
            mat = np.dot(rotY, mat)
            mat = np.dot(rotZ, mat)

            mat[0][0] += ox
            mat[1][0] += oy
            mat[2][0] += oz

            result[i][j] = np.zeros(3, np.float64)
            result[i][j][0] = mat[0][0]
            result[i][j][1] = mat[1][0]
            result[i][j][2] = mat[2][0]

    return result


def translate_one(vec, rx, ry, rz):
    return translate([[vec]], rx, ry, rz, 0, 0, 0, 1, 1, 1, 1)[0][0]
