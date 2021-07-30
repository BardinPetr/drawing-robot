from time import sleep
from control.camera import Camera
from control.manipulator import ManipulatorControl


def draw(contours, image_width, image_height, width, height):
    cam = Camera()

    mc = ManipulatorControl("192.168.13.2", man_tool_speed=0.5, man_tool_acc=0.3, activate_gripper=False)
    
    normal = mc.full_calibration(cam)

    sleep(1)
    mc.calibrate_distance()
    sleep(1)
    
    data_m = mc.prepare_contours(contours, width, height, image_width, image_height)
    # data_m = list(filter(lambda x: len(x) > 10, data_m))
    #for i in data_m:
    #    print("Contour len", len(i))
    #    mc.draw_contour(i, force=0.5)
    end_x = data_m[0][0][0]
    end_y = data_m[0][0][1]
    end_z = data_m[0][0][2]
    while len(data_m) > 0:
        cont = data_m[0]
        cont_dist = (end_x - cont[0][0])**2 + (end_y - cont[0][1])**2 + (end_z - cont[0][2])**2
        index = 0
        j = 0
        for i in data_m:
            cont_dist_new = (end_x - i[0][0])**2 + (end_y - i[0][1])**2 + (end_z - i[0][2])**2
            if cont_dist_new < cont_dist:
                cont = i
                cont_dist = (end_x - i[0][0])**2 + (end_y - i[0][1])**2 + (end_z - i[0][2])**2
                index = j
            j += 1
        
        print("Contour len", len(cont))
        mc.draw_contour(cont, force=0.1)
        end_x = cont[len(cont)-1][0]
        end_y = cont[len(cont)-1][1]
        end_z = cont[len(cont)-1][2]
        data_m.pop(index)   
        
    mc.pen_start(dist=0.05)
    