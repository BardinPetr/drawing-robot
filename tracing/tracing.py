import cv2
import numpy as np
import shutil


def tracing(filename, shading=False):
    frame = cv2.imread("C:/images/" + filename)
    gauss_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gauss_frame = cv2.GaussianBlur(gauss_frame, (7, 7), 0)
    (w, h) = gauss_frame.shape
    for i in range(w):
        for j in range(h):
            gauss_frame[i][j] = (gauss_frame[i][j] // 50)
            gauss_frame[i][j] *= 50

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv_frame = cv2.GaussianBlur(hsv_frame, (9, 9), 0)
    (w, h, d) = frame.shape
    for i in range(w):
        for j in range(h):
            gray_frame[i][j] = ((hsv_frame[i][j][2] << 8) + hsv_frame[i][j][0])/256

    edge_frame = cv2.Canny(gray_frame, 0, 30, L2gradient=True)
    connected = cv2.connectedComponentsWithStats(edge_frame, 8, cv2.CV_32S)
    labels = connected[1]
    stats = connected[2]
    for i in range(w):
        for j in range(h):
            cur_label = labels[i][j]
            if stats[cur_label][cv2.CC_STAT_AREA] < 20:
                edge_frame[i][j] = 0

    contours, hierarchy = cv2.findContours(edge_frame, 1, cv2.CHAIN_APPROX_SIMPLE)

    if shading:
        shading_frame = edge_frame.copy()
        for i in range(w):
            for j in range(h):
                shading_frame[i][j] = 0

        for i in range(0, h, 5):
            for j in range(w):
                color = gauss_frame[j][i]
                if color <= 50:
                    edge_frame[j][i] = 255
                    shading_frame[j][i] = 255
                if color <= 100 and i % 4 == 0:
                    edge_frame[j][i] = 255
                    shading_frame[j][i] = 255
                # if color <= 150 and i % 8 == 0:
                #     edge_frame[j][i] = 255
                #     shading_frame[j][i] = 255

        connected = cv2.connectedComponentsWithStats(shading_frame, 8, cv2.CV_32S)
        labels = connected[1]
        stats = connected[2]
        for i in range(w):
            for j in range(h):
                cur_label = labels[i][j]
                if stats[cur_label][cv2.CC_STAT_AREA] < 10:
                    edge_frame[i][j] = 0
                    shading_frame[i][j] = 0

        shading_contours, hierarchy = cv2.findContours(shading_frame, 1, cv2.CHAIN_APPROX_SIMPLE)

        routes = [0] * (len(contours) + len(shading_contours))
        for i in range(len(contours)):
            routes[i] = [0] * len(contours[i])
            for j in range(len(contours[i])):
                routes[i][j] = np.arange(2)
                routes[i][j][0] = contours[i][j][0][0]
                routes[i][j][1] = contours[i][j][0][1]

        for i in range(len(shading_contours)):
            routes[i + len(contours)] = [0] * len(shading_contours[i])
            for j in range(len(shading_contours[i])):
                routes[i + len(contours)][j] = np.arange(2)
                routes[i + len(contours)][j][0] = shading_contours[i][j][0][0]
                routes[i + len(contours)][j][1] = shading_contours[i][j][0][1]
    else:
        routes = [0] * len(contours)
        for i in range(len(contours)):
            routes[i] = [0] * len(contours[i])
            for j in range(len(contours[i])):
                routes[i][j] = np.arange(2)
                routes[i][j][0] = contours[i][j][0][0]
                routes[i][j][1] = contours[i][j][0][1]

    edge_frame = cv2.bitwise_not(edge_frame)
    cv2.imwrite("C:/images/preview.jpg", edge_frame)
    shutil.copy("C:/images/preview.jpg", "C:/Users/Михаил/PycharmProjects/upload_server/static/preview.jpg")
    return routes
