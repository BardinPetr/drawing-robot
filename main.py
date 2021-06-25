from control.camera import Camera
from control.manipulator import ManipulatorControl

# from control.platfrom import Platform

mc = ManipulatorControl("192.168.13.2", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)
# p = Platform('192.168.12.20')
cam = Camera()

print("CONNECTED")

# mc.move_to_home()

# mc.move_to_start()
#
# # exit(0)
# sleep(5)
# mc.rtde_ctrl.teachMode()
# print(1)
# sleep(6)
# mc.rtde_ctrl.endTeachMode()
# print(2)
# sleep(1)
from time import sleep

color, depth, _ = cam.capture()
_, _, res_data = cam.detect_planes(color.data, depth.data)

normal = Camera.get_main_normal(res_data)

mc.align_perpendicular(normal)
start_pos = mc.get_pos_full()

mc.calibrate_distance()
sleep(1)

mc.pen_up()
sleep(1)

mc.pen_down()
sleep(1)

mc.pen_up()
sleep(1)

