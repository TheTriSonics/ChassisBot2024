from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from drivetrain import Drivetrain

class DriveToPoint(CommandBase):
    def __init__(self, drive: Drivetrain, gyro, x, y, targetHeading):
        super().__init__()
        self.drive = drive
        self.gyro = gyro
        self.x = x
        self.y = y
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
        currx = pose.X()
        curry = pose.Y()
        xPower = self.xPID.calculate(currx, self.x)
        yPower = self.yPID.calculate(curry, self.y)
        rotPower = self.rotPID.calculate(curr_yaw, self.targetHeading)
        self.drive.drive(xPower, yPower, rotPower)

    def end(self, i):
        pass

    def isFinished(self):
        from math import sqrt
        pose = self.drive.getPose()
        currx = pose.X()
        curry = pose.Y()
        deltax = currx - self.x
        deltay = curry - self.y
        total_distance = sqrt(deltax*deltax + deltay*deltay)
        SmartDashboard.putNumber('dtp err', total_distance)
        if total_distance < 5/100:
            return True
        return False

        return False