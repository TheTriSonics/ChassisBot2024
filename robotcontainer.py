import typing
import commands2
import commands2.button
import wpilib


class RobotContainer:

    curr_x: float = 0
    curr_y: float = 0

    def __init__(self) -> None:
        self._configure_robot_hardware()
        self._configure_controllers()
        pass

    def _configure_robot_hardware(self) -> None:
        self.drivetrain = None

    def _configure_controllers(self) -> None:
        self.driver = wpilib.Joystick(0)

    def systime() -> float:
        from time import time
        return time()

    def update_odometry(self) -> None:
        pass

    def get_auton_command(self) -> typing.Optional[commands2.CommandBase]:
        return None

    def reset_odometry(self) -> None:
        self.curr_x = 0
        self.curr_y = 0
