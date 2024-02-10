from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from subsystems.drivetrain import Drivetrain

class Rotate(CommandBase):
    def __init__(self, drive: Drivetrain, gyro, targetHeading):
        super().__init__()
        self.drive = drive
        self.gyro = gyro
        self.targetHeading = targetHeading
        p, i, d = 3, 0.45, 0
        self.xPID = PIDController(p, i, d)
        self.yPID = PIDController(p, i, d)
        self.rotPID = PIDController(0.10, 0, 0)
    
    def initialize(self):
        SmartDashboard.putString("dtp" , "i AM the dtp")

    def execute(self):
        pose = self.drive.getPose()
        curr_yaw = self.gyro.get_yaw()
        rotPower = self.rotPID.calculate(curr_yaw, self.targetHeading)
        self.drive.drive(0, 0, rotPower)

    def end(self, i):
        pass

    def isFinished(self):
        from math import sqrt
        pose = self.drive.getPose()
        currRot = pose.rotation().degrees()
        deltaRot = currRot - self.targetHeading
        SmartDashboard.putNumber('dtp err', deltaRot)
        if deltaRot < 2:
            return True
        return False

        return False