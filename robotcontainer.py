import typing
import commands2
import commands2.button
import wpilib
from subsystems.drivetrain import SwerveDrivetrain
from subsystems.imu import IMU


class RobotContainer:

    curr_x: float = 0
    curr_y: float = 0
    gyro: IMU
    drivetrain: SwerveDrivetrain

    def __init__(self) -> None:
        self._configure_robot_hardware()
        self._configure_controllers()
        pass

    # This will prevent us from making more than once instance of this class
    # at runtime. We've turned the class into a 'singleton' with this where
    # the instance of the class will be stored in the module level 'instance'
    # variable. Once defined we'll just return that same copy any time code
    # attempts to construct the RobotContainer() again.
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(RobotContainer, cls).__new__(cls)
        return cls.instance

    def _configure_robot_hardware(self) -> None:
        self.drivetrain = SwerveDrivetrain()
        self.gyro = IMU()

    def _configure_controllers(self) -> None:
        pass
        #self.driver = wpilib.Joystick(0)

    def systime(self) -> float:
        from time import time
        return time()

    def update_odometry(self) -> None:
        pass

    def get_auton_command(self) -> typing.Optional[commands2.CommandBase]:
        return None

    # You can add other methods here to return commands that accomplish other
    # tasks.

    def reset_odometry(self) -> None:
        self.curr_x = 0
        self.curr_y = 0
