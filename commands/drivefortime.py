import commands2
from time import time
from drivetrain import Drivetrain
from wpilib import SmartDashboard


class DriveForTime(commands2.CommandBase):

    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drive = drive
        self.power = 100
        self.addRequirements(drive)

    def initialize(self):
        self.start = time()

    def execute(self):
        self.drive.drive(self.power, 0, 0, False, 0.02)
        SmartDashboard.putNumber("power", self.power)
        # self.power += 1

    def end(self, i):
        self.drive.lockWheels()
        pass

    def isFinished(self):
        now = time()
        return (now - self.start) > 2
