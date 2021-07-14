from functools import reduce
from time import sleep, time
from pprint import pprint
import numpy as np
import rtde_control
import rtde_receive
from scipy.spatial.transform import Rotation as R

from utils.robotiq_gripper import RobotiqGripper
from utils.transform import rv2rpy, rpy2rv, rotation_from_vectors, translate, translate_one

import cv2
import math


class ManipulatorControl:
    last_joints_pos = None
    last_tcp_pos = None

    plane_orient = None
    plane_normal = None
    plane_touch = None

    def __init__(self, manipulator_ip,
                 man_speed=1, man_acc=1.4,
                 man_tool_speed=0.25, man_tool_acc=1.2,
                 grip_speed=70, grip_force=100,
                 activate_gripper=True):
        self.ip = manipulator_ip # установка настроек манипулятора

        self.man_speed = man_speed
        self.man_acc = man_acc
        self.grip_speed = grip_speed
        self.grip_force = grip_force
        self.man_tool_speed = man_tool_speed
        self.man_tool_acc = man_tool_acc

        self.gripper = RobotiqGripper() # подключение захвата
        self.gripper.connect(manipulator_ip, 63352)
        if activate_gripper:
            self.gripper.activate()

        self.rtde_recv = rtde_receive.RTDEReceiveInterface(manipulator_ip, []) # подключение интерфейса чтения манипулятора
        self.rtde_ctrl = rtde_control.RTDEControlInterface(manipulator_ip) # подключение интерфейса управления манипулятора

    def __del__(self):
        self.rtde_ctrl.disconnect() # отключение от манипулятора
        self.rtde_recv.disconnect()

    def get_pos_full(self): # получение расположения и угла поворота TCP, угол в виде rotation vector
        res = self.rtde_recv.getActualTCPPose()
        if self.last_tcp_pos == res:
            self.check_conn()
        self.last_tcp_pos = res
        return np.array(res)

    def get_pos(self): # получение расположения TCP в метрах
        return self.get_pos_full()[:3]

    def get_rot(self, as_rv=False): # получение угла поворота, as_rv - в виде rotation vector 
        rv = self.get_pos_full()[3:]
        return np.array(rv if as_rv else rv2rpy(rv))

    def get_joints(self): # получение текущих углов сочленений
        res = self.rtde_recv.getActualQ()
        if self.last_joints_pos == res:
            self.check_conn()
        self.last_joints_pos = res
        return res

    def move_joints(self, pos): # переместить сочленения в заданные позиции
        return self.rtde_ctrl.moveJ(pos, self.man_speed, self.man_acc)

    def move_joints_rel(self, diff): # переместить сочленения относительно текущего положения
        q_cur = self.get_joints()
        q_new = [q_cur[i] + np.radians(diff[i]) for i in range(6)]
        return self.move_joints(q_new)

    def move_tool(self, pos, do_async=False): # переместить TCP, задание углов наклона необязательно
        pos = pos if len(pos) == 6 else [*pos, *self.get_rot(True)]
        return self.rtde_ctrl.moveL(pos, self.man_tool_speed, self.man_tool_acc, do_async)

    def move_tool_rel(self, diff, do_async=False): # переместить TCP относительно текущего положения
        diff = diff if len(diff) == 6 else [*diff, 0, 0, 0]
        p_cur = self.get_pos_full()
        p_new = [p_cur[i] + diff[i] for i in range(len(p_cur))]
        return self.move_tool(p_new, do_async)

    def until_contact(self, vel): # двигаться до контакта с заданной скоростью
        self.rtde_ctrl.moveUntilContact([*vel, 0, 0, 0])
        sleep(1)
        return self.get_pos()

    def set_speed(self, vel): # задать скорость перемещений
        self.rtde_ctrl.speedL(vel)

    def grip(self, state): # открыть/закрыть захват
        self.gripper.move_and_wait_for_pos(
            self.gripper.get_closed_position() if state else self.gripper.get_open_position(),
            self.grip_speed,
            self.grip_force
        )

    def check_conn(self): # проверка подключения
        if not self.rtde_recv.isConnected():
            self.rtde_recv.reconnect()
        if not self.rtde_ctrl.isConnected():
            self.rtde_ctrl.reconnect()
            # self.rtde_ctrl.reuploadScript()

    def move_to_start(self):
        self.move_joints_rel([90, 0, 0, 0, -180, 0])
        sleep(1)
        self.move_joints_rel([0, 40, -20, 20, 0, 0])
        sleep(1)
        self.move_tool_rel([0.1, 0, -0.1, 0, 0, 0])

    def move_to_home(self):
        self.rtde_ctrl.moveL([0.127, 0.351, 0.193, 3.96, 0.13, -0.04])

    def normal_to_target_pos(self, normal, as_rv=False): # вычисление углов для выравнивания по нормали
        rot_d = self.get_rot()
        res = R.from_euler("xyz", rot_d, True).apply(normal) * -1

        target = -rotation_from_vectors([0, 0, 1], res).as_euler('xyz', True)

        if abs(rot_d[2] - target[2]) > 100:
            target[2] = 180 - abs(target[2])

        for i in range(3):
            if abs(rot_d[i] - target[i]) > 90:
                target[i] *= -1

        print("CURRENT", *rot_d)
        print("TARGET", *target)

        return rpy2rv(target) if as_rv else target

    def align_perpendicular(self, normal): # выравнивание по перпендикуляру
        target = self.normal_to_target_pos(normal, as_rv=True)

        # self.plane_normal = normal / np.linalg.norm(normal)
        #cur_rot = self.get_rot()
        cur_rot = rv2rpy(target)
        self.plane_normal = translate_one((0.0, 0.0, 1.0), *cur_rot)
        print(self.plane_normal)
        self.plane_orient = target

        cur = self.get_pos()
        IK = self.rtde_ctrl.getInverseKinematics([*cur, *target],[])
        IK[5] = 0;
        res = self.move_joints(IK)
        # res = self.move_tool([*cur, *target])
        return res

    def _calibrate_in_move(self, cam, vel, time_target=2):
        normals = []
        self.set_speed(vel)
        start_time = time()
        while time() - start_time < time_target:
            new_n = cam.basic_get_normal()
            normals.append(new_n)
        self.set_speed([0, 0, 0, 0, 0, 0])
        self.rtde_ctrl.speedStop(5)
        return normals

    def full_calibration(self, cam): # полная калибровка нормали
        self.align_perpendicular(cam.basic_get_normal())
        normals = reduce(lambda x, i: x + self._calibrate_in_move(cam, i, 2), [
            [0, 0, 0, 0.1, 0, 0],
            [0, 0, 0, -0.1, 0, 0],
            [0, 0, 0, -0.1, 0, 0],
            [0, 0, 0, 0.1, 0, 0],
            [0, 0, 0, 0, 0.1, 0],
            [0, 0, 0, 0, -0.1, 0],
            [0, 0, 0, 0, -0.1, 0],
            [0, 0, 0, 0, 0.1, 0]
        ], [])
        normal = np.average(np.array(normals), axis=0)
        self.align_perpendicular(normal)
        
        while self.rtde_recv.getAsyncOperationProgress() > -1:
            pass
            
        joints = self.get_joints()
        joints[5] = -math.radians(100)
        self.move_joints(joints)
        
        angles = [0]
        flag = True
        
        while flag:
            while self.rtde_recv.getAsyncOperationProgress() > -1:
                pass
            
            sleep(2)
            image_capture, _, _ = cam.capture()
            frame = cv2.cvtColor(image_capture.data, 1)
            
            #print(frame)
            #qrDecoder = cv2.QRCodeDetector()
            #data, bbox, rectifiedCode = qrDecoder.detectAndDecode(frame)
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            
            if markerIds != None:
                #dx = int(bbox[0][2][0]) - int(bbox[0][1][0])
                #dy = int(bbox[0][2][1]) - int(bbox[0][1][1])
                dx = int(corners[0][0][2][0]) - int(corners[0][0][1][0])
                dy = int(corners[0][0][2][1]) - int(corners[0][0][1][1])
                angle = math.degrees(math.atan2(dx, dy))
                print(angle)
                self.move_joints_rel([0, 0, 0, 0, 0, -angle])
                flag = False
            else:
                print("aruco-code not found")
                joints = self.get_joints()
                joints[5] += math.radians(30)
                if joints[5] < math.radians(100):
                    self.move_joints(joints)
                else:
                    flag = False
        
        '''sleep(3)
        
        image_capture, _, _ = cam.capture()
        frame = cv2.cvtColor(image_capture.data, 1)
        cv2.imwrite("/home/main/image.jpg", frame)'''
        self.plane_orient = self.get_rot(as_rv=True)
        #dist = cam.basic_get_normal_dist()
        dist = 0.29
        
        image_capture, _, _ = cam.capture()
        frame = cv2.cvtColor(image_capture.data, 1)
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        if markerIds != None:
            x = (int(corners[0][0][2][1]) - 1280/2)
            y = -(int(corners[0][0][2][0]) - 720/2)
            X = dist*math.tan(math.radians(90)/2)*x/(1280)
            Y = dist*math.tan(math.radians(59)/2)*y/(720)
            print("X: " + str(X) + "Y: " + str(Y))
            cur_rot = self.get_rot()
            offset = translate_one((Y + 0.04, X + 0.04, 0.0), *cur_rot)
            #self.move_tool_rel(offset)
        
        '''cur_rot = self.get_rot()
        offset = translate_one((0.04, 0.04, 0.0), *cur_rot)
        self.move_tool_rel(offset)'''
        print("dist: " + str(dist))
        return normal

    def calibrate_distance(self): # калибровка касанием плоскости
        if self.plane_normal is None:
            raise Exception("Not selected plane")

        self.plane_touch = self.until_contact(self.plane_normal * 0.02)
        self.pen_up()

    def to_plane_contact(self):
        self.move_tool(self.plane_touch)

    def pen_down(self): # опустить карандаш
        if self.plane_normal is None:
            raise Exception("Not selected plane")
        self.until_contact(self.plane_normal * 0.04)
        # self.move_tool_rel(self.plane_normal * dist)

    def pen_start(self, dist=0.02): # переместить карандаш к точке старта
        if self.plane_normal is None:
            raise Exception("Not selected plane")
        self.move_tool([*(self.plane_touch - self.plane_normal * dist), *self.plane_orient])
        
    def pen_up(self, dist=0.01): # поднять карандаш
        if self.plane_normal is None:
            raise Exception("Not selected plane")
        self.move_tool([*(self.get_pos() - self.plane_normal * dist), *self.plane_orient])

    def prepare_contours(self, data, width, height, pixel_width, pixel_height): # перевод координат из СО рисунка в СО манипулятора
        orient = rv2rpy(self.plane_orient)
        return translate(
            data,
            *orient,
            *self.plane_touch,
            width, height, pixel_width, pixel_height,
            inverse=False
        )

    def draw_contour(self, data, dt=1.0 / 500, force=1): # рисование контура
        # self.pen_up()
        start = [[*(data[0] - self.plane_normal * 0.01), *self.plane_orient, 0.3, 0.2, 0]]
        self.rtde_ctrl.moveL(start, True)
        
        while self.rtde_recv.getAsyncOperationProgress() > -1:
            pass
        
        self.rtde_ctrl.moveUntilContact([*(self.plane_normal * 0.01), 0, 0, 0])
        
        '''for i in range(1000):
            start = time()
            self.rtde_ctrl.forceMode([0, 0, 0, *self.plane_orient],
                                     [0, 0, 1, 0, 0, 0],
                                     [0, 0, force, 0, 0, 0],
                                     1,
                                     [100, 100, 100, 1, 1, 1])
            end = time()
            duration = end - start
            if duration < dt:
                sleep(dt - duration)'''
        
        pos = [[*i, *self.plane_orient, 0.3, 0.2, 0] for i in data]
        #pos = [[*i, *self.plane_orient, 0.5, 0.4, 0] for i in data]
        self.rtde_ctrl.moveL(pos, True)
        #sleep(1)
        while self.rtde_recv.getAsyncOperationProgress() > -1:
            start = time()
            self.rtde_ctrl.forceMode([0, 0, 0, *self.plane_orient],
                                     [0, 0, 1, 0, 0, 0],
                                     [0, 0, force, 0, 0, 0],
                                     1,
                                     [100, 100, 100, 1, 1, 1])
            end = time()
            duration = end - start
            if duration < dt:
                sleep(dt - duration)

        self.rtde_ctrl.forceModeStop()
        self.pen_up()


if __name__ == "__main__":
    mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)

    while True:
        print("%0.5f %0.5f %0.5f %0.5f %0.5f %0.5f" % (*list(mc.rtde_recv.getActualTCPPose()),))
