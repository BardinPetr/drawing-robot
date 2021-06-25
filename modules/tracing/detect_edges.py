import cv2 as cv
import numpy as np
import tensorflow as tf
from src.model import DexiNed


def build():
    global model
    model = DexiNed([0, 0, 0])
    model.build((None, 512, 512, 3))
    model.load_weights("src/DexiNed23_model.h5")
    # model.summary()


def detectEdges(img):
    w, h = img.shape[:2]
    if w > h:
        wn = 512
        hn = int(512 * (h / w))
    else:
        wn = int(512 * (w / h))
        hn = 512

    global model
    model.rgbn_mean = img.mean(axis=0).mean(axis=0)
    # model.rgbn_mean = [103.939,116.779,123.68]
    # model.rgbn_mean[0], model.rgbn_mean[2] = model.rgbn_mean[2], model.rgbn_mean[0]

    transformed_img = np.zeros((512, 512, 3,), dtype=np.uint8)
    transformed_img[:hn, :wn] = cv.resize(img, (wn, hn))

    pred = model.call(np.array([transformed_img], dtype=np.float64))

    preds = []
    for tmp_p in pred:
        preds.append(np.reshape(tf.sigmoid(tmp_p), (512, 512)))

    res = (preds[6] * 2 + preds[5] + preds[4] + preds[3]) / 5

    output = np.zeros((hn, wn,))
    output[0:hn - 1, 0:wn - 1] = res[0:hn - 1, 0:wn - 1]

    return np.array(cv.resize(output, (h, w)) * 255, dtype=np.uint8)


def processEdges(edges):
    edges = 255 - edges

    edges = cv.dilate(edges, np.ones((2, 2,)))
    edges = 255 - edges
    edges = (edges > 15) * edges
    edges = 255 - edges

    edges = 255 - cv.adaptiveThreshold(edges, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)

    # edges = cv.ximgproc.thinning(edges, None, cv.ximgproc.THINNING_GUOHALL)

    contours, hierarchy = cv.findContours(edges,
                                          cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    output = np.zeros(edges.shape, dtype=np.uint8)

    cv.drawContours(output, contours, -1, 255, 1)

    return output, contours


build()

if __name__ == "__main__":
    img = cv.imread("data/sea.jpg")
    img = cv.resize(img, None, fx=0.7, fy=0.7)

    res = 255 - detectEdges(img)
    res = np.reshape(res, (res.shape[0], res.shape[1], 1))
    res = np.concatenate((res, res, res), axis=2)

    output = np.concatenate((img, res), axis=1)
    cv.imshow("Result", output)

    cv.waitKey(0)
