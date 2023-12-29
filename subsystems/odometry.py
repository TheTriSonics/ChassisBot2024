import commands2
from typing import Optional
from ctre.sensors import Pigeon2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d

from robotcontainer import RobotContainer


class Odometry(commands2.SubsystemBase):
    pose: Pose2d
    INCHES_PER_METER: float = 39.36

    def get_pose(self):
        return self.pose

    def periodic(self) -> None:
        cont = RobotContainer()
        # TODO: This is where a limelight camera can be used to augment
        # wheel odometery / dead reckoning
        self.pose = cont.drivetrain.odometry.getPose()
        SmartDashboard.putNumber("X Location", self.pose.X())
        SmartDashboard.putNumber("Y Location", self.pose.Y())
        SmartDashboard.putNumber("Heading",
                                 self.pose.rotation().degrees().value())
