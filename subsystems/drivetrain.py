#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import ntcore
import subsystems.swervemodule as swervemodule
from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import (
    SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry,
    SwerveModulePosition, ChassisSpeeds
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
)

kMaxSpeed = 4.8  # m/s
kMaxAngularSpeed = math.pi * 5

swerve_offset = 30 / 100  # cm converted to meters


class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self, gyro) -> None:
        super().__init__()
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = Translation2d(-swerve_offset, -swerve_offset)

        self.frontLeft = swervemodule.SwerveModule(12, 22, 32, False, 'Front left')
        self.frontRight = swervemodule.SwerveModule(11, 21, 31, True, 'Front right')
        self.backLeft = swervemodule.SwerveModule(14, 24, 34, False, 'Back left')
        self.backRight = swervemodule.SwerveModule(13, 23, 33, True, 'Back right')

        self.ntinst = ntcore.NetworkTableInstance.getDefault().getTable('limelight')
        self.ll_json = self.ntinst.getStringTopic("json")
        self.ll_json_entry = self.ll_json.getEntry('[]')

        self.fieldRelative = False

        self.gyro = gyro

        self.cs = None

        self.kinematics = SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading_rotation_2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.resetOdometry()

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(1.9, 0.0, 0.0), # Translation PID constants
                PIDConstants(1.6, 0.0, 0.0), # Rotation PID constants
                kMaxSpeed, # Max module speed, in m/s.
                0.431, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def llJson(self) -> str:
        return self.ll_json.getEntry("[]")

    def getSpeeds(self):
        self.cs = self.kinematics.toChassisSpeeds([self.frontLeft.getState(), self.frontRight.getState(), self.backLeft.getState(), self.backRight.getState()])
        # if self.cs is None:
        #     return wpimath.kinematics.ChassisSpeeds(0,0,0)
        SmartDashboard.putNumber("csvx", self.cs.vx)
        SmartDashboard.putNumber("csvy", self.cs.vy)

        return self.cs

    def driveRobotRelative(self, speeds):
        self.fieldRelative = False
        # self.drive(speeds.vx, speeds.vy, speeds.omega)
        # SmartDashboard.putNumber("vx", speeds.vx)
        # SmartDashboard.putNumber("vy", speeds.vy)
        # SmartDashboard.putNumber("omega", speeds.omega)
        # self.cs = speeds
        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds, Translation2d(0, 0))

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])
        pass

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def resetOdometry(self, pose: Pose2d = None):
        if pose is None:
            self.gyro.set_yaw(0)
            defaultPos = (
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
                SwerveModulePosition(0, Rotation2d(0)),
            )
            self.odometry.resetPosition(
                Rotation2d(), defaultPos, Pose2d()
            )
        else:
            self.gyro.set_yaw(pose.rotation().degrees())
            self.odometry.resetPosition(
                pose.rotation(),
                modulePositions=[
                    self.frontLeft.getPosition(),
                    self.frontRight.getPosition(),
                    self.backLeft.getPosition(),
                    self.backRight.getPosition(),
                ],
                pose=pose,
            )

    def get_heading_rotation_2d(self) -> Rotation2d:
        return Rotation2d(math.radians(self.gyro.get_yaw()))

    def toggleFieldRelative(self):
        self.fieldRelative = not self.fieldRelative

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        periodSeconds: float = 0.02,
    ) -> None:
        SmartDashboard.putBoolean("Fr", self.fieldRelative)
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative
        :      to the field.
        :param periodSeconds: Time
        """
        if self.fieldRelative:
            self.cs = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.get_heading_rotation_2d(),
                )
        else:
            self.cs = ChassisSpeeds(xSpeed, ySpeed, rot)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(self.cs, Translation2d(0, 0))
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def lockWheels(self):
        for sm in [self.frontLeft, self.frontRight,
                   self.backLeft, self.backRight]:
            sm.lock()

    def setStates(self, fl: SwerveModuleState, fr: SwerveModuleState,
                  bl: SwerveModuleState, br: SwerveModuleState):
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(bl)
        self.backRight.setDesiredState(br)

    def getAngles(self) -> tuple[float, float, float, float]:
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
        SmartDashboard.putNumber("heading",
                                 self.get_heading_rotation_2d().degrees())

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()
