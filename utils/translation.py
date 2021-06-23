import numpy as np
import math


def translation(mas, rx, ry, rz, ox, oy, oz, width, height, pixel_width, pixel_height):
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

            result[i][j] = np.arange(3)
            result[i][j][0] = mat[0][0]
            result[i][j][1] = mat[1][0]
            result[i][j][2] = mat[2][0]

    return result
