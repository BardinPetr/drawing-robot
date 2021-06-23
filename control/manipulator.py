import rtde_control
import rtde_receive

from utils.robotiq_gripper import RobotiqGripper


class ManipulatorControl:
    last_joints_pos = None
    last_tcp_pos = None

    def __init__(self, manipulator_ip,
                 man_speed=1, man_acc=1.4,
                 man_tool_speed=0.25, man_tool_acc=1.2,
                 grip_speed=100, grip_force=100,
                 activate_gripper=True):
        self.ip = manipulator_ip

        self.man_speed = man_speed
        self.man_acc = man_acc
        self.grip_speed = grip_speed
        self.grip_force = grip_force
        self.man_tool_speed = man_tool_speed
        self.man_tool_acc = man_tool_acc

        self.gripper = RobotiqGripper()
        self.gripper.connect(manipulator_ip, 63352)
        if activate_gripper:
            self.gripper.activate()

        self.rtde_recv = rtde_receive.RTDEReceiveInterface(manipulator_ip, [])
        self.rtde_ctrl = rtde_control.RTDEControlInterface(manipulator_ip)

    def get_pos(self):
        res = self.rtde_recv.getActualTCPPose()
        if self.last_tcp_pos == res:
            self.check_conn()
        self.last_tcp_pos = res
        return res

    def get_joints(self):
        res = self.rtde_recv.getActualQ()
        if self.last_joints_pos == res:
            self.check_conn()
        self.last_joints_pos = res
        return res

    def move_joints(self, pos):
        self.rtde_ctrl.moveJ(pos, self.man_speed, self.man_acc)

    def move_joints_rel(self, diff):
        q_cur = self.get_joints()
        q_new = [q_cur[i] + diff[i] for i in range(6)]
        self.move_joints(q_new)

    def move_tool(self, pos):
        self.rtde_ctrl.moveL(pos, self.man_tool_speed, self.man_tool_acc)

    def move_tool_rel(self, diff):
        p_cur = self.get_pos()
        p_new = [p_cur[i] + diff[i] for i in range(len(p_cur))]
        self.move_tool(p_new)

    def until_contact(self, vel):
        self.rtde_ctrl.moveUntilContact([*vel, 0, 0, 0])
        return self.get_pos()

    def set_speed(self, vel):
        self.rtde_ctrl.speedL(vel)

    def grip(self, state):
        self.gripper.move_and_wait_for_pos(
            self.gripper.get_closed_position() if state else self.gripper.get_open_position(),
            self.grip_speed,
            self.grip_force
        )

    def check_conn(self):
        if not self.rtde_recv.isConnected():
            self.rtde_recv.reconnect()
        if not self.rtde_ctrl.isConnected():
            self.rtde_ctrl.reconnect()
            # self.rtde_ctrl.reuploadScript()


if __name__ == "__main__":
    mc = ManipulatorControl("192.168.12.245", man_tool_speed=0.3, man_tool_acc=0.3, activate_gripper=False)

    while True:
        print("%0.5f %0.5f %0.5f %0.5f %0.5f %0.5f" % (*list(mc.rtde_recv.getActualTCPPose()),))
        # mc.move_tool_rel([0, 0, 0, 0.1, 0, 0])
        # mc.move_tool_rel([0.05, 0, 0.05, 0, 0, 0])
        # mc.grip(True)
        # sleep(1)
        # mc.grip(False)
        # mc.check_conn()
        # mc.until_contact([0, 0, -0.1])
        # sleep(3)
        # mc.move_tool_rel([0, 0, 0, -0.1, 0, 0])
        # mc.move_tool_rel([0.05, 0, -0.05, 0, 0, 0])
        # mc.until_contact([0, -0.4, 0])
        # sleep(0.01)
