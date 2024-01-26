#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpimath.geometry
import wpimath.kinematics
import swervemodule
from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard

# TODO: Set to a real value in centimeters per second
kMaxSpeed = 200.0
kMaxAngularSpeed = math.pi*4


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = wpimath.geometry.Translation2d(30, 30)
        self.frontRightLocation = wpimath.geometry.Translation2d(30, -30)
        self.backLeftLocation = wpimath.geometry.Translation2d(-30, 30)
        self.backRightLocation = wpimath.geometry.Translation2d(-30, -30)

        self.frontLeft = swervemodule.SwerveModule(12, 22, 32, 'Front left')
        self.frontRight = swervemodule.SwerveModule(11, 21, 31, 'Front right')
        self.backLeft = swervemodule.SwerveModule(14, 24, 34, 'Back left')
        self.backRight = swervemodule.SwerveModule(13, 23, 33, 'Back right')

        defaultPos = [
            wpimath.kinematics.SwerveModulePosition(0, wpimath.geometry.Rotation2d(0)),
            wpimath.kinematics.SwerveModulePosition(0, wpimath.geometry.Rotation2d(0)),
            wpimath.kinematics.SwerveModulePosition(0, wpimath.geometry.Rotation2d(0)),
            wpimath.kinematics.SwerveModulePosition(0, wpimath.geometry.Rotation2d(0)),
        ]

        self.gyro = Pigeon2(41, "rio")

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.odometry.resetPosition(
            wpimath.geometry.Rotation2d(), defaultPos, wpimath.geometry.Pose2d()
        )

        self.gyro.set_yaw(0)

    def get_heading_rotation_2d(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(math.radians(self.gyro.get_yaw().value))

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        SmartDashboard.putBoolean("Fr", fieldRelative)
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative
        :      to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.get_heading_rotation_2d(),
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])
    
    def lockWheels(self):
        for sm in [self.frontLeft, self.frontRight, self.backLeft, self.backRight]:
            sm.lock()

    def setStates(self,fl,fr,bl,br):
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(bl)
        self.backRight.setDesiredState(br)

    def getAngles(self):
        flAng = self.frontLeft.getState().angle.radians()
        frAng = self.frontRight.getState().angle.radians()
        blAng = self.backLeft.getState().angle.radians()
        brAng = self.backRight.getState().angle.radians()
        return flAng, frAng, blAng, brAng
    
    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        pose = self.odometry.getPose()
        SmartDashboard.putNumber("x", pose.X())
        SmartDashboard.putNumber("y", pose.Y())
        SmartDashboard.putNumber("heading",self.get_heading_rotation_2d().degrees())