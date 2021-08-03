from tempfile import NamedTemporaryFile

import cv2
import numpy as np

from modules.tracing.detect_edges import processEdges, detectEdges
from modules.tracing.filling import fill


def autoscale(img):
    w, h = img.shape[:2]
    if w > h:
        wn = 512
        hn = 512 * (h / w)
    else:
        hn = 512
        wn = 512 * (w / h)
    return cv2.resize(img, (int(hn), int(wn)))


def trace_img(file):
    img = cv2.imread("/home/main/drawing-robot-upload/" + file)
    img = autoscale(img)

    pic_edges, contours_main = processEdges(detectEdges(img))
    for i in range(len(contours_main)):
        contours_main[i] = np.reshape(contours_main[i], (contours_main[i].shape[0], contours_main[i].shape[2]))

    clearing_area = 255 - cv2.dilate(pic_edges, np.ones((7, 7)))

    contours_filling = [] #fill(img, clearing_area != 0)

    res = np.zeros(img.shape[:2], dtype=np.uint8)
    for contour in contours_filling + contours_main:
        for i in range(contour.shape[0] - 1):
            cv2.line(res, (contour[i][0], contour[i][1]), (contour[i + 1][0], contour[i + 1][1]), 255, thickness=1)

    res = 255 - res

    output = np.concatenate((img, np.stack((res, res, res), axis=2)), axis=1)

    f = NamedTemporaryFile("w", suffix='.jpg', delete=False)
    cv2.imwrite(f.name, output)
    return contours_filling, contours_main, f, output
