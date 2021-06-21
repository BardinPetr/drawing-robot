import pickle

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

mtx, dist, _, _ = pickle.load(open('cal.pkl', 'rb'))

crt = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

sz = (6, 7)


def find_cbc(data, size, criteria):
    r, c = cv2.findChessboardCorners(data, size)
    if r:
        cv2.cornerSubPix(data, c, size, (-1, -1), criteria)
        return c
    return None


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


while True:
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = find_cbc(gray, sz, crt)

    try:
        if corners is not None:
            _, rot_v, tr_v = cv2.solvePnP(objp, corners, mtx, dist)
            img_points, _ = cv2.projectPoints(axis, rot_v, tr_v, mtx, dist)
            img = draw(img, corners, img_points)
            cv2.drawChessboardCorners(img, sz, corners, True)
    except:
        pass

    cv2.imshow('img', img)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
