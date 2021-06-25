from time import sleep

from control.camera import Camera
from control.manipulator import ManipulatorControl
from modules.tracer import trace_img

# from control.platfrom import Platform
# p = Platform('192.168.12.20')

cam = Camera()

mc = ManipulatorControl("192.168.13.2",
                        man_tool_speed=0.3, man_tool_acc=0.3,
                        activate_gripper=False)

print("CONNECTED")

# print()
# exit()
# mc.move_to_start()

# print("Give a pen")
# sleep(3)
# mc.grip(True)
# sleep(1)

normal = mc.full_calibration(cam)
# mc.align_perpendicular(cam.basic_get_normal())
sleep(1)
mc.calibrate_distance()
sleep(1)

cntr_f, cntr_m, file, img = trace_img("hello.jpg")
print(file.name, 0.10, 0.10 * img.shape[1] / img.shape[0])
# exit(0)
# data_f = mc.prepare_contours(
#     cntr_f,
#     img.shape[0]/512, img.shape[1]/512,
#     *img.shape[:2]
# )
data_m = mc.prepare_contours(
    cntr_m,
    0.10, 0.10 * img.shape[1] / img.shape[0],
    # img.shape[0] / 9680 * 2, img.shape[1]  / 9680 * 2,
    *img.shape[:2]
)
data_m = list(filter(lambda x: len(x) > 10, data_m))
for i in data_m:
    print("Contour len", len(i))
    mc.draw_contour(i, force=0.1)
    sleep(1)

# for i in data_f:
#     mc.draw_contour(i, force=0.1)
#     sleep(1)

exit(0)
# from control.manipulator import ManipulatorControl
#
# mc = ManipulatorControl("192.168.13.2",
#                         man_tool_speed=0.3, man_tool_acc=0.4,
#                         activate_gripper=False)
#
# task_frame = [0, 0, 0, 0, 0, 0]
# selection_vector = [0, 0, 1, 0, 0, 0]
# wrench_down = [0, 0, 1, 0, 0, 0]
# # wrench_up = [0, 0, 10, 0, 0, 0]
# force_type = 1
# limits = [2, 2, 1.5, 1, 1, 1]
# dt = 1.0 / 1000  # 2ms
# joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]
#
# # Move to initial joint position with a regular moveJ
# # mc.rtde_ctrl.moveJ(joint_q)
# import time
#
# # Execute 500Hz control loop for 4 seconds, each cycle is 2ms
# for i in range(5000):
#     start = time.time()
#     # First move the robot down for 2 seconds, then up for 2 seconds
#     # if i > 1000:
#     #     mc.rtde_ctrl.forceMode(task_frame, selection_vector, wrench_up, force_type, limits)
#     # else:
#     mc.rtde_ctrl.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
#     end = time.time()
#     duration = end - start
#     if duration < dt:
#         time.sleep(dt - duration)
#
# mc.rtde_ctrl.forceModeStop()
# mc.rtde_ctrl.stopScript()
