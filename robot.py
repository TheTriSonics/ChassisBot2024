#!/usr/bin/env python3
#chloe thomsen
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# All units of length will be in centimeters or converted from another unit
# to centimeters for internal use.a (Leading 0's on meters can be hard to
# keep track of, so we'll use centimeters instead)

import json
import wpilib
import commands2
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import subsystems.drivetrain as drivetrain 
from wpilib import SmartDashboard
from commands.rotate import Rotate
from commands.drivefordistance import DriveForDistance
from commands.haltdrive import HaltDrive
from commands.drivetopoint import DriveToPoint
from subsystems.gyro import Gyro
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.commands import FollowPathHolonomic
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

 
class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        # self.controller = wpilib.XboxController(0)
        self.controller = wpilib.Joystick(0)
        
        self.gyro = Gyro()
        self.swerve = drivetrain.Drivetrain(self.gyro)

        # Slew rate limiters to make joystick inputs more gentle
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(2)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(2)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(1)

    '''
    def followPathCommand(self, pathName: str):
        path = PathPlannerPath.fromPathFile(pathName)
        return FollowPathHolonomic(
            path,
            self.swerve.getPose, # Robot pose supplier
            self.swerve.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.swerve.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
                4.5, # Max module speed, in m/s
                0.4, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.swerve.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
        '''

    def autonomousInit(self):
        # cmd = DriveForDistance(self.swerve, 50)
        # cmd = HaltDrive(self.swerve)
        self.swerve.resetOdometry()
        self.gyro.set_yaw(45)
        cmd = PathPlannerAuto("Goofy")
        # cmd = Rotate(self.swerve, self.gyro, 0)
        haltcmd = HaltDrive(self.swerve)
        scg = commands2.SequentialCommandGroup([cmd, haltcmd])
        scg.schedule()
        """
        drive1 = DriveToPoint(self.swerve, self.gyro, 200, 100, 180)
        halt1 = HaltDrive(self.swerve)
        drive2 = DriveToPoint(self.swerve, self.gyro, 0, 0, 0)
        halt2 = HaltDrive(self.swerve)
        scg = commands2.SequentialCommandGroup([drive1, halt1, drive2, halt2])
        scg.schedule()
        """
        pass

    def autonomousPeriodic(self) -> None:
        # self.driveWithJoystick(False)
        self.swerve.updateOdometry()
        SmartDashboard.putNumber("yaw", self.gyro.get_yaw())

    def teleopPeriodic(self) -> None:
        if self.controller.getRawButtonPressed(2):
            self.swerve.toggleFieldRelative()
        
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()
        SmartDashboard.putNumber("yaw", self.gyro.get_yaw())
        currx, curry = self.getVisionXY()
        
        if currx is not None and curry is not None:
            SmartDashboard.putNumber('vx', currx)
            SmartDashboard.putNumber('vy', curry)

    def getVisionXY(self):
        data = self.swerve.ll_json_entry.get()
        obj = json.loads(data)
        totalx, totaly = 0, 0
        currx, curry = None, None
        if len(obj) > 0 and 'Results' in obj.keys():
            obj = obj['Results']
            if 'Fiducial' in obj.keys():
                obj = obj['Fiducial']
                targets = len(obj)
                pp = json.dumps(obj, indent=4)
                for f in obj:
                    totalx += f['tx']
                    totaly += f['ty']
                if targets > 0:
                    currx = totalx / targets
                    curry = totaly / targets
        return currx, curry

    def getVisionXY(self):
        data = self.swerve.ll_json_entry.get()
        obj = json.loads(data)
        totalx, totaly = 0, 0
        currx, curry = None, None
        if len(obj) > 0 and 'Results' in obj.keys():
            obj = obj['Results']
            if 'Fiducial' in obj.keys():
                obj = obj['Fiducial']
                targets = len(obj)
                pp = json.dumps(obj, indent=4)
                for f in obj:
                    totalx += f['tx']
                    totaly += f['ty']
                if targets > 0:
                    currx = totalx / targets
                    curry = totaly / targets
        return currx, curry
                    
    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = -(
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), 0.04)
            )

        )
        xsign = 1 if xSpeed > 0 else -1
        xSpeed = xSpeed * xSpeed * xsign * drivetrain.kMaxSpeed

        ySpeed = -(
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), 0.04)
            )

        )
        ysign = 1 if ySpeed > 0 else -1
        ySpeed = ySpeed * ySpeed * ysign * drivetrain.kMaxSpeed
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(4), 0.04)
            )

        )
        rotsign = 1 if rot > 0 else -1
        rot = rot * rot * rotsign * drivetrain.kMaxAngularSpeed
        SmartDashboard.putNumber('xspeed', xSpeed)
        SmartDashboard.putNumber('yspeed', ySpeed)
        SmartDashboard.putNumber('rot', rot)
        self.swerve.drive(xSpeed, ySpeed, rot, self.getPeriod())

    def disabledPeriodic(self) -> None:
        SmartDashboard.putNumber("FL encoder", self.swerve.frontLeft.driveMotor.get_position().value)