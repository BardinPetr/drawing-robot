import cv2 as cv
import numba
import numpy as np
from skimage.segmentation import slic


@numba.njit
def seq(arr):
    begin = 0
    end = 0
    ans = []
    prev = False
    for i in range(len(arr)):
        curr = arr[i]

        if prev and not curr:
            ans.append((begin, end))

        if curr:
            end += 1
        else:
            begin = i
            end = i

        prev = curr

    if end != begin:
        ans.append((begin, end))

    return ans


def fill(image, clearingArea):
    image = np.array(image, dtype=np.float32) / 255
    segments = (slic(image, n_segments=400, sigma=5) + 1) * clearingArea

    return fill_opt(image, segments)


def fill_opt(image, segments):
    N = np.max(segments)

    contours = []

    for i in range(1, N):
        mask = (segments == i)
        mean = 255 * np.array(cv.mean(image, mask=np.array(mask, dtype=np.uint8) * 255)[:3])
        mean = np.reshape(np.array(mean, dtype=np.uint8), (1, 1, 3))
        mean = cv.cvtColor(mean, cv.COLOR_BGR2LAB)[0, 0, 0]
        mean = 255 - mean

        step = 3 / (float(mean) / 255) + 1.5
        pattern = np.zeros(image.shape[:2], dtype=np.uint8)

        for p in range(0, pattern.shape[0], int(step)):
            if np.sum(mask[p, :]) == 0:
                continue

            res = seq(mask[p, :])

            for (begin, end) in res:
                cv.line(pattern, (begin, p), (end, p), 255, thickness=1)
                contours.append((np.array([np.array([begin, p]), np.array([end, p])])))

    return contours
