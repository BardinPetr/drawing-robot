from subprocess import run
from tempfile import NamedTemporaryFile, TemporaryDirectory
from time import sleep

import cv2
import k4a
import numpy as np


class Camera:
    def __init__(self):
        self.device = k4a.Device.open()

        if self.device is None:
            raise Exception("Couldn't connect to Kinect")

        device_config = k4a.DeviceConfiguration(
            color_format=k4a.EImageFormat.COLOR_BGRA32,
            color_resolution=k4a.EColorResolution.RES_720P,
            depth_mode=k4a.EDepthMode.WFOV_2X2BINNED,
            camera_fps=k4a.EFramesPerSecond.FPS_5
        )
        self.device.start_cameras(device_config)

        self.calibration = self.device.get_calibration(
            depth_mode=device_config.depth_mode,
            color_resolution=device_config.color_resolution
        )

        self.transform = k4a.Transformation(self.calibration)

    def __del__(self):
        try:
            self.device.stop_cameras()
        except:
            pass

    def capture(self):
        res = self.device.get_capture(-1)
        if res is None:
            print("Capture failed")
            sleep(0.5)
            return self.capture()
        depth = self.transform.depth_image_to_color_camera(res.depth)
        # cloud = self.transform.depth_image_to_point_cloud(depth, k4a.ECalibrationType.DEPTH).data.reshape(-1, 3)
        return res.color, depth, None

    def get_intrinsics(self):
        intrinsics = self.calibration.depth_cam_cal.intrinsics.parameters.param
        return intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy

    @staticmethod
    def detect_planes(img_color, img_depth):
        with NamedTemporaryFile('w', suffix='.jpg') as c_file, \
                NamedTemporaryFile('w', suffix='.png') as d_file, \
                TemporaryDirectory() as res_dir:
            cv2.imwrite(c_file.name, img_color)
            cv2.imwrite(d_file.name, img_depth)

            res = run(["/home/main/RGBDPlaneDetection/build/RGBDPlaneDetection", c_file.name, d_file.name, res_dir],
                      capture_output=True)
            if res.returncode != 0:
                return None
            res_stdout = res.stdout.decode("utf-8").split("\n")[1:]

            res_img = cv2.imread(res_dir + "/0-plane.png")

            with open(res_dir + "/0-plane-label.txt", 'r') as f:
                n = int(f.readline())
                res_labels = np.array([[int(j) for j in i.split()] for i in f.readlines()])

            with open(res_dir + "/0-plane-data.txt", 'r') as f:
                _ = f.readline()
                res_data = [{
                    "i":      int(i[0]),
                    "size":   int(i[1]),
                    "color":  np.array([int(j) for j in i[2:5]]),
                    "normal": np.array([float(j) for j in i[5:8]]),
                    "center": np.array([float(j) for j in i[8:11]]),
                    "dist":   float(res_stdout.pop(0).split()[-1])
                } for i in [f.readline().split() for _ in range(n)]]

            return res_img, res_labels, res_data

    @staticmethod
    def get_main_normal(data):
        return max(data, key=lambda x: x["size"])['normal']

    def basic_get_normal(self):
        color, depth, _ = self.capture()
        _, _, res_data = self.detect_planes(color.data, depth.data)
        return Camera.get_main_normal(res_data)


if __name__ == "__main__":
    # mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)

    c = Camera()
    while True:
        color, depth, _ = c.capture()
        res_img, res_labels, res_data = c.detect_planes(color.data, depth.data)
        m_plane = max(res_data, key=lambda x: x["size"])
        print(m_plane["normal"])
