#!/usr/bin/env python3
#
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
import drivetrain
from wpilib import SmartDashboard
from commands.drivefortime import DriveForTime
from commands.drivefordistance import DriveForDistance
from commands.haltdrive import HaltDrive
from commands.drivetopoint import DriveToPoint
from subsystems.gyro import Gyro

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

    def autonomousInit(self):
        # cmd = DriveForDistance(self.swerve, 50)
        # cmd = HaltDrive(self.swerve)
        self.swerve.resetOdometry()
        drive = DriveToPoint(self.swerve, self.gyro, 200, 100, 180)
        halt = HaltDrive(self.swerve)
        scg = commands2.SequentialCommandGroup([drive, halt])
        scg.schedule()

    def autonomousPeriodic(self) -> None:
        # self.driveWithJoystick(False)
        self.swerve.updateOdometry()
        SmartDashboard.putNumber("yaw", self.gyro.get_yaw())

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)
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
                    
    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = -(
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), 0.02)
            )

        )
        xsign = 1 if xSpeed > 0 else -1
        xSpeed = xSpeed * xSpeed * xsign * drivetrain.kMaxSpeed

        ySpeed = -(
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), 0.02)
            )

        )
        ysign = 1 if ySpeed > 0 else -1
        ySpeed = ySpeed * ySpeed * ysign * drivetrain.kMaxSpeed
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(4), 0.02)
            )

        )
        rotsign = 1 if rot > 0 else -1
        rot = rot * rot * rotsign * drivetrain.kMaxAngularSpeed
        SmartDashboard.putNumber('xspeed', xSpeed)
        SmartDashboard.putNumber('yspeed', ySpeed)
        SmartDashboard.putNumber('rot', rot)
        self.swerve.drive(xSpeed, ySpeed, rot, False, self.getPeriod())
