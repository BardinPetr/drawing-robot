from time import sleep

import pygame

from control.manipulator import ManipulatorControl
from control.platfrom import Platform

mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3)
p = Platform('192.168.12.20')

pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
clk = pygame.time.Clock()

base_speed = 0.5
flag = True

print(1)

while p.ros.is_connected:
    if True:
        for event in pygame.event.get():
            pass

        vel = [joy.get_axis(i) * base_speed for i in range(3)]
        if any(map(lambda x: abs(x) > 0.05, vel)):
            p.drive(-vel[1], vel[2])
            flag = True
        elif flag:
            flag = False
            p.drive(0, 0)
    sleep(0.01)
