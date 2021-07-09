from time import sleep
from control.camera import Camera
from control.manipulator import ManipulatorControl


def draw(contours, image_width, image_height, width, height):
    cam = Camera()

    mc = ManipulatorControl("192.168.13.2", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)
    
    normal = mc.full_calibration(cam)

    sleep(1)
    mc.calibrate_distance()
    sleep(1)
    
    data_m = mc.prepare_contours(contours, width, height, image_width, image_height)
    # data_m = list(filter(lambda x: len(x) > 10, data_m))
    for i in data_m:
        print("Contour len", len(i))
        mc.draw_contour(i, force=4)
        
    mc.pen_start(dist=0.05)
    