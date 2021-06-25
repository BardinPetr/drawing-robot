import pickle

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

crt = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)


def find_cbc(data, size, criteria):
    r, c = cv2.findChessboardCorners(data, sz)
    if r:
        # cv2.cornerSubPix(data, c, size, (-1, -1), criteria)
        return c
    return None


sz = (6, 7)

imgpoints = []

while True:
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = find_cbc(gray, sz, crt)

    if corners is not None:
        imgpoints.append(corners)
        cv2.drawChessboardCorners(img, sz, corners, True)

    # while True:pass

    cv2.imshow('img', img)
    if cv2.waitKey(1) == 27:
        break

    if len(imgpoints) > 5:
        break

objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
objpoints = [objp] * len(imgpoints)

ret, *res = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
pickle.dump(res, open('cal.pkl', 'wb'))

cap.release()
cv2.destroyAllWindows()
