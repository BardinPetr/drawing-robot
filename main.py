from time import sleep

import pygame

from control.manipulator import ManipulatorControl
from utils.rotvec import rv2rpy, rpy2rv

mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)
# p = Platform('192.168.12.20')

pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
clk = pygame.time.Clock()

base_speed = 10
flag = True

print(1)
# sleep(3)
mc.rtde_ctrl.teachMode()
# sleep(3)

while True:
    for event in pygame.event.get():
        pass

    vel = [joy.get_axis(i) * base_speed for i in range(3)]
    # if any(map(lambda x: abs(x) > 0.1, vel)):
    #     mc.set_speed([0, 0, 0, *rpy2rv(vel[1], vel[2], vel[0])])
    # else:
    #     mc.set_speed([0] * 6)


    pos = mc.get_pos()
    rpy = rv2rpy(*pos[3:])
    print((' '.join(["%0.5f"] * 3)) % (*rpy, ))
    # print(' '.join(["%0.5f"] * 6) % (*list(mc.get_pos()),))
    mc.check_conn()  # equal to sleep(0.02)
    # mc.rtde_ctrl.teachMode()
    sleep(0.1)

mc.rtde_ctrl.endTeachMode()
mc.rtde_ctrl.stopScript()

# while p.ros.is_connected:
#     if True:
#         for event in pygame.event.get():
#             pass
#
#         vel = [joy.get_axis(i) * base_speed for i in range(3)]
#         if any(map(lambda x: abs(x) > 0.05, vel)):
#             p.drive(-vel[1], vel[2])
#             flag = True
#         elif flag:
#             flag = False
#             p.drive(0, 0)
#     sleep(0.01)
