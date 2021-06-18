from robolink import *
from robodk import *
import pygame
from time import sleep

from robolink import *
from robodk import *

RDK = Robolink()

robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
# robot.setConnectionParams('192.168.2.35', 30001, '/', 'anonymous', '')

assert robot.ConnectSafe() == ROBOTCOM_READY
RDK.setRunMode(RUNMODE_RUN_ROBOT)


# robot.setSpeed(300)

# joints = robot.Joints()
# target = robot.Pose()
# pos = target.Pos()

def move(dxyz, speed=30):
    move = mult3(dxyz, speed)

    pos = robot.SolveFK(robot.Joints())

    new_pos = transl(move) * pos
    new_joints = robot.SolveIK(new_pos)

    if len(new_joints.tolist()) < 6:
        return

    robot.MoveJ(new_joints)

    """
    new_target = Mat(target)
    pos = new_target.Pos()
    pos = [pos[i] + d[i] for i in range(3)]
    new_target.setPos(pos)
    robot.MoveL(new_target)
    """


pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
clk = pygame.time.Clock()

while True:
    for event in pygame.event.get():
        pass

    vel = [joy.get_axis(i) for i in range(3)]
    move(vel)
    # clk.tick(20)

pygame.quit()
