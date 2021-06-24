def align_perpendicular(mc, cam):
    color, depth, _ = cam.capture()
    _, _, res_data = cam.detect_planes(color.data, depth.data)
    normal = cam.get_main_normal(res_data)
    target = mc.normal_to_target_pos(normal, as_rv=True)
    cur = mc.get_pos()[:3]
    return mc.move_tool([*cur, *target])
