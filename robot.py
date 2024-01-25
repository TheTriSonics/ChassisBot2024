#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# All units of length will be in centimeters or converted from another unit
# to centimeters for internal use.a (Leading 0's on meters can be hard to
# keep track of, so we'll use centimeters instead)

import wpilib
import commands2
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
from wpilib import SmartDashboard
from commands.drivefortime import DriveForTime


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        # self.controller = wpilib.XboxController(0)
        self.controller = wpilib.Joystick(0)
        self.swerve = drivetrain.Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle
       
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(2)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(2)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(1)

    def autonomousInit(self):
        cmd = DriveForTime(self.swerve)
        cmd.schedule()

    def autonomousPeriodic(self) -> None:
        # self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)
        self.swerve.updateOdometry()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), 0.02)
            )
            
        )
        xsign = 1 if xSpeed > 0 else -1
        xSpeed = xSpeed * xSpeed * xsign* drivetrain.kMaxSpeed

        ySpeed = (
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), 0.02)
            )
            
        )
        ysign = 1 if ySpeed > 0 else -1
        ySpeed = ySpeed * ySpeed * ysign* drivetrain.kMaxSpeed
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
