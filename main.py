from time import sleep

from control.camera import Camera
from control.manipulator import ManipulatorControl
# from control.platfrom import Platform
from soft_modules.by_camera import align_perpendicular

mc = ManipulatorControl("192.168.13.2", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)
# p = Platform('192.168.12.20')
cam = Camera()

mc.rtde_ctrl.teachMode()
print(1)
sleep(5)
mc.rtde_ctrl.endTeachMode()
print(2)
align_perpendicular(mc, cam)
