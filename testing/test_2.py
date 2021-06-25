import math

import numpy as np
from scipy.spatial.transform import Rotation as R

from control.manipulator import ManipulatorControl
from utils.rotvec import rpy2rv


def cam_to_base(x, y, z): return z, x, -y
def base_to_cam(x, y, z): return y, -z, x

mc = ManipulatorControl("192.168.13.2", activate_gripper=False)
# p = Platform('192.168.12.20')

vec = np.array([0, -1 / 2, -np.sqrt(3) / 2])
vec = cam_to_base(*vec)
print(vec)

cur_pos = mc.get_pos()
print(cur_pos)
cur_rot = R.from_rotvec(cur_pos[3:])
print(cur_rot.as_euler('xyz', True))

vec_t = cur_rot.apply(vec)

print(vec_t)

print(R.from_rotvec(vec_t).as_euler("xyz", True))

print([*cur_pos[:3],
       *rpy2rv(math.atan2(vec_t[1], vec_t[2]), math.atan2(vec_t[0], vec_t[2]), math.atan2(vec_t[0], vec_t[1]), )])

# mc.move_tool([*cur_pos[:3], *rpy2rv(math.atan2(vec_t[1], vec_t[2]), math.atan2(vec_t[0], vec_t[2]), math.atan2(vec_t[0], vec_t[1]), )])


# rpy = rv2rpy(*mc.get_pos()[3:])
#
# print(rpy)
#
# v_t = translate_one(vec, rpy[2], rpy[0], rpy[1])
# print(np.linalg.norm(vec))
# print(np.linalg.norm(v_t))
# rpy_target = rv2rpy(*v_t)
#
# print(v_t, rpy_target)
