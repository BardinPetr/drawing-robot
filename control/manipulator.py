from time import sleep

import numpy as np
import rtde_control
import rtde_receive
from scipy.spatial.transform import Rotation as R

from control.camera import Camera
from utils.robotiq_gripper import RobotiqGripper
from utils.transform import rv2rpy, rpy2rv, rotation_from_vectors


class ManipulatorControl:
    last_joints_pos = None
    last_tcp_pos = None

    plane_orient = None
    plane_normal = None
    plane_touch = None

    def __init__(self, manipulator_ip,
                 man_speed=1, man_acc=1.4,
                 man_tool_speed=0.25, man_tool_acc=1.2,
                 grip_speed=100, grip_force=100,
                 activate_gripper=True):
        self.ip = manipulator_ip

        self.man_speed = man_speed
        self.man_acc = man_acc
        self.grip_speed = grip_speed
        self.grip_force = grip_force
        self.man_tool_speed = man_tool_speed
        self.man_tool_acc = man_tool_acc

        self.gripper = RobotiqGripper()
        self.gripper.connect(manipulator_ip, 63352)
        if activate_gripper:
            self.gripper.activate()

        self.rtde_recv = rtde_receive.RTDEReceiveInterface(manipulator_ip, [])
        self.rtde_ctrl = rtde_control.RTDEControlInterface(manipulator_ip)

    def __del__(self):
        self.rtde_ctrl.disconnect()
        self.rtde_recv.disconnect()

    def get_pos_full(self):
        res = self.rtde_recv.getActualTCPPose()
        if self.last_tcp_pos == res:
            self.check_conn()
        self.last_tcp_pos = res
        return res

    def get_pos(self):
        return self.get_pos_full()[:3]

    def get_rot(self, as_rv=False):
        rv = self.get_pos_full()[3:]
        return np.array(rv if as_rv else rv2rpy(rv))

    def get_joints(self):
        res = self.rtde_recv.getActualQ()
        if self.last_joints_pos == res:
            self.check_conn()
        self.last_joints_pos = res
        return res

    def move_joints(self, pos):
        return self.rtde_ctrl.moveJ(pos, self.man_speed, self.man_acc)

    def move_joints_rel(self, diff):
        q_cur = self.get_joints()
        q_new = [q_cur[i] + np.radians(diff[i]) for i in range(6)]
        return self.move_joints(q_new)

    def move_tool(self, pos):
        pos = pos if len(pos) == 6 else [*pos, *self.get_rot(True)]
        print(pos)
        return self.rtde_ctrl.moveL(pos, self.man_tool_speed, self.man_tool_acc)

    def move_tool_rel(self, diff):
        diff = diff if len(diff) == 6 else [*diff, 0, 0, 0]
        p_cur = self.get_pos_full()
        p_new = [p_cur[i] + diff[i] for i in range(len(p_cur))]
        return self.move_tool(p_new)

    def until_contact(self, vel):
        self.rtde_ctrl.moveUntilContact([*vel, 0, 0, 0])
        return self.get_pos()

    def set_speed(self, vel):
        self.rtde_ctrl.speedL(vel)

    def grip(self, state):
        self.gripper.move_and_wait_for_pos(
            self.gripper.get_closed_position() if state else self.gripper.get_open_position(),
            self.grip_speed,
            self.grip_force
        )

    def check_conn(self):
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

    def normal_to_target_pos(self, normal, as_rv=False):
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

    def align_perpendicular(self, normal):
        target = self.normal_to_target_pos(normal, as_rv=True)

        self.plane_normal = normal / np.linalg.norm(normal)
        self.plane_orient = target

        cur = self.get_pos()
        res = self.move_tool([*cur, *target])

        return res

    def calibrate_distance(self):
        if self.plane_normal is None:
            raise Exception("Not selected plane")

        self.plane_touch = self.until_contact(self.plane_normal * 0.1)

    def to_plane_contact(self):
        self.move_tool(self.plane_touch)

    def pen_down(self, dist=0.05):
        if self.plane_normal is None:
            raise Exception("Not selected plane")
        self.move_tool_rel(self.plane_normal * dist)

    def pen_up(self, dist=0.05):
        if self.plane_normal is None:
            raise Exception("Not selected plane")
        self.move_tool_rel(self.plane_normal * -dist)


if __name__ == "__main__":
    mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)

    while True:
        print("%0.5f %0.5f %0.5f %0.5f %0.5f %0.5f" % (*list(mc.rtde_recv.getActualTCPPose()),))
