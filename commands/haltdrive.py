import commands2
from time import time
from drivetrain import Drivetrain
from wpilib import SmartDashboard
import wpimath.kinematics
import math
class HaltDrive(commands2.CommandBase):

    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drive = drive
        self.addRequirements(drive)      

    def initialize(self):
       pass

    def execute(self):
        defaultState = [
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(-math.pi/4)),
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(math.pi/4)),
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(math.pi/4)),
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(-math.pi/4)),
        ]
        self.drive.setStates(defaultState[0], defaultState[1], defaultState[2], defaultState[3])
        
    def end(self, i):
        SmartDashboard.putString("done", "done again")
        self.drive.lockWheels()
        pass

    def isFinished(self):
        fl, fr, bl, br = self.drive.getAngles()
        error = 0
        error += abs(fl-(-math.pi/4))
        error += abs(fr-(math.pi/4))
        error += abs(bl-(math.pi/4))
        error += abs(br-(-math.pi/4))
        SmartDashboard.putNumber("angleerror", math.degrees(error))
        return error < math.radians(4)