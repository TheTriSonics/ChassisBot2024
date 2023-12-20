import typing
import wpilib
import commands2
from robotcontainer import RobotContainer


class ChassisBot(commands2.TimedCommandRobot):

    auton_command: typing.Optional[commands2.CommandBase] = None

    def robotInit(self) -> None:
        self.container = RobotContainer()
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        pass


if __name__ == '__main__':
    wpilib.run(ChassisBot)
