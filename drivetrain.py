#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import ntcore
import wpimath.geometry
import wpimath.kinematics
import swervemodule
from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveModuleState

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

# TODO: Set to a real value in centimeters per second
kMaxSpeed = 3.5 # m/s
kMaxAngularSpeed = math.pi*4.5 

swerve_offset = 30 / 100  # cm converted to meters


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def llJson(self) -> str:
        return self.ll_json.getEntry("[]")

    def __init__(self, gyro) -> None:
        # TODO: Set these to the right numbers in centimeters
        self.frontLeftLocation = wpimath.geometry.Translation2d(swerve_offset, swerve_offset)
        self.frontRightLocation = wpimath.geometry.Translation2d(swerve_offset, -swerve_offset)
        self.backLeftLocation = wpimath.geometry.Translation2d(-swerve_offset, swerve_offset)
        self.backRightLocation = wpimath.geometry.Translation2d(-swerve_offset, -swerve_offset)

        self.frontLeft = swervemodule.SwerveModule(12, 22, 32, 'Front left')
        self.frontRight = swervemodule.SwerveModule(11, 21, 31, 'Front right')
        self.backLeft = swervemodule.SwerveModule(14, 24, 34, 'Back left')
        self.backRight = swervemodule.SwerveModule(13, 23, 33, 'Back right')

        self.ntinst = ntcore.NetworkTableInstance.getDefault().getTable('limelight')
        self.ll_json = self.ntinst.getStringTopic("json")
        self.ll_json_entry = self.ll_json.getEntry('[]')

        self.fieldRelative = False    

        self.gyro = gyro

        self.cs = None

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

        self.resetOdometry()

        p, i, d = 25, 0.045, 0

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(p, i, d), # Translation PID constants
                PIDConstants(15.0, 0.0, 0.0), # Rotation PID constants
                kMaxSpeed, # Max module speed, in m/s.
                0.431, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def getSpeeds(self):
        if self.cs is None:
            return wpimath.kinematics.ChassisSpeeds(0,0,0)
        SmartDashboard.putNumber("csvx", self.cs.vx)
        SmartDashboard.putNumber("csvy", self.cs.vy)
        return self.cs

    def driveRobotRelative(self, speeds):
        self.fieldRelative = False
        # self.drive(speeds.vx, speeds.vy, speeds.omega)
        SmartDashboard.putNumber("vx", speeds.vx)
        SmartDashboard.putNumber("vy", speeds.vy)
        SmartDashboard.putNumber("omega", speeds.omega)
        self.cs = speeds
        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds, wpimath.geometry.Translation2d(0, 0))
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
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
        if pose == None:
            self.gyro.set_yaw(0)
            defaultPos = (
                wpimath.kinematics.SwerveModulePosition(0, Rotation2d(0)),
                wpimath.kinematics.SwerveModulePosition(0, Rotation2d(0)),
                wpimath.kinematics.SwerveModulePosition(0, Rotation2d(0)),
                wpimath.kinematics.SwerveModulePosition(0, Rotation2d(0)),
            )
            self.odometry.resetPosition(
                Rotation2d(), defaultPos, wpimath.geometry.Pose2d()
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
            self.cs = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.get_heading_rotation_2d(),
                )
        else:
            self.cs = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
        swerveModuleStates = self.kinematics.toSwerveModuleStates(self.cs, wpimath.geometry.Translation2d(0, 0))
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
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
