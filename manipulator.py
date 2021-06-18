from time import sleep

from utils.robotiq_gripper import RobotiqGripper


class ManipulatorControl:
    gripper_speed = 100
    gripper_force = 100

    def __init__(self, manipulator_ip):
        self.ip = manipulator_ip

        self.gripper = RobotiqGripper()
        self.gripper.connect(manipulator_ip, 63352)
        self.gripper.activate()

    def grip(self, state):
        self.gripper.move_and_wait_for_pos(
            self.gripper.get_closed_position() if state else self.gripper.get_open_position(),
            self.gripper_speed,
            self.gripper_force
        )


if __name__ == "__main__":
    mc = ManipulatorControl("192.168.12.245")

    while True:
        mc.grip(True)
        sleep(1)
        mc.grip(False)
        sleep(1)
