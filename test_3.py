from time import sleep

import cv2
import numpy as np

from control.manipulator import ManipulatorControl

mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)
cap = cv2.VideoCapture(2)

sz = (4, 4)
crt = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.01)


def run():
    img_half_size = np.array(cap.read()[1].shape[1::-1]) / 2

    while True:
        _, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        r, corners = cv2.findChessboardCorners(gray, sz)
        if r:
            cv2.cornerSubPix(gray, corners, sz, (-1, -1), crt)
            cv2.drawChessboardCorners(img, sz, corners, True)

            corners = corners.reshape(-1, 2)
            center = np.average(corners, axis=0)
            # cv2.circle(img, tuple(center), 4, (255, 0, 0), 3)

            diff = (img_half_size - center) / img_half_size

            print(diff)
            if abs(diff[0]) < 0.1 and abs(diff[1]) < 0.1:
                print("Done")
                mc.set_speed([0, 0, 0, 0, 0, 0])
                sleep(3)
                mc.until_contact([0, 0, 0.1])
                exit(0)

            spd = diff / 90
            mc.set_speed([-spd[0], spd[1], 0, 0, 0, 0])

        cv2.imshow('img', img)
        if cv2.waitKey(1) == 27:
            mc.set_speed([0, 0, 0, 0, 0, 0])
            exit(0)


try:
    run()
finally:
    mc.set_speed([0, 0, 0, 0, 0, 0])
    cap.release()
    cv2.destroyAllWindows()
