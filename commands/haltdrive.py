import commands2
import math

from wpilib import SmartDashboard
from drivetrain import Drivetrain
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState


class HaltDrive(commands2.CommandBase):

    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self):
        pass

    def execute(self):
        defaultState = [
            SwerveModuleState(0, Rotation2d(-math.pi/4)),
            SwerveModuleState(0, Rotation2d(math.pi/4)),
            SwerveModuleState(0, Rotation2d(math.pi/4)),
            SwerveModuleState(0, Rotation2d(-math.pi/4)),
        ]
        self.drive.setStates(defaultState[0], defaultState[1],
                             defaultState[2], defaultState[3])

    def end(self, i):
        SmartDashboard.putString("halt", "done again")
        self.drive.lockWheels()

    def isFinished(self):
        fl, fr, bl, br = self.drive.getAngles()
        error = 0
        error += abs(fl-(-math.pi/4))
        error += abs(fr-(math.pi/4))
        error += abs(bl-(math.pi/4))
        error += abs(br-(-math.pi/4))
        SmartDashboard.putNumber("angleerror", math.degrees(error))
        return error < math.radians(4)
