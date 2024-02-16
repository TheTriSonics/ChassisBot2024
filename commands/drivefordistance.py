import commands2
from time import time
from subsystems.mechanical.drivetrain import Drivetrain
from wpilib import SmartDashboard


class DriveForDistance(commands2.CommandBase):

    def __init__(self, drive: Drivetrain, distance: float):
        super().__init__()
        self.drive = drive
        self.power = 100
        self.distance = distance
        self.addRequirements(drive)

    def initialize(self):
        curr_pose = self.drive.odometry.getPose()
        self.start_x = curr_pose.X()
        self.start = time()

    def execute(self):
        self.drive.drive(self.power, 0, 0, False, 0.02)
        SmartDashboard.putNumber("power", self.power)
        # self.power += 1

    def end(self, i):
        SmartDashboard.putString("done", "done again")
        self.drive.lockWheels()
        pass

    def isFinished(self):
        curr_pose = self.drive.odometry.getPose()
        return (curr_pose.X() - self.start_x) > self.distance
