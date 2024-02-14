#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
from wpimath.kinematics import (
    SwerveModuleState, SwerveModulePosition
)
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import DutyCycleOut, VelocityDutyCycle
from phoenix6 import configs, signals
from wpilib import SmartDashboard


kModuleMaxAngularVelocity = math.pi*10
kModuleMaxAngularAcceleration = math.tau
kWheelRadius = 0.0508  # m
# kGearRatio = 7.131

# encoder_to_mech_ratio =


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        canCoderChannel: int,
        inverted: bool,
        name: str,
    ) -> None:
        self.name = name
        """Constructs a SwerveModule.

        :param driveMotorChannel:    CAN ID for the drive motor.
        :param turningMotorChannel:  CAN ID for the turning motor.
        :param canCoderChannel:      CAN ID for the turning motor's encoder.
        :param name:                 Name when displayed on Dashboard
        """
        self.driveMotor = TalonFX(driveMotorChannel)
        self.turningMotor = TalonFX(turningMotorChannel)

        driveConfigurator = self.driveMotor.configurator

        drive_Output = configs.MotorOutputConfigs()
        if inverted:
            drive_Output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        else:
            drive_Output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE

        drive_feedback = configs.FeedbackConfigs()
        drive_feedback.with_sensor_to_mechanism_ratio(7.131)
        driveConfigurator.apply(drive_feedback)

        drive_PID = configs.Slot0Configs()
        drive_PID.k_p = 0.16
        drive_PID.k_s = 0.045
        drive_PID.k_v = 0.09
        driveConfigurator.apply(drive_PID)

        turnConfigurator = self.turningMotor.configurator
        turn_motor_configs = configs.MotorOutputConfigs()
        turn_motor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        turn_motor_configs.neutral_mode = signals.NeutralModeValue.BRAKE
        turnConfigurator.apply(turn_motor_configs)

        self.turnEncoder = CANcoder(canCoderChannel)

        self.drivePIDController = PIDController(0.24, 0, 0)
        self.turningPIDController = PIDController(0.25, 0, 0)

        # Limit the PID Controller's input range between -pi and pi and
        # set the input to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)
        self.driveMotor.set_position(0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        speed = (
            self.driveMotor.get_velocity().value * (2 * math.pi * kWheelRadius)
        )
        # SmartDashboard.putNumber(f'{self.name} speed', speed)
        rot = Rotation2d(
            self.turnEncoder.get_absolute_position().value * math.tau
        )
        return SwerveModuleState(speed, rot)

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.:46


        """
        pos = self.driveMotor.get_position().value
        # Now convert from rotation units to centimeters
        # distance = pos * math.tau * kWheelRadius
        distance = pos * (2 * math.pi * kWheelRadius) # convert rotations to cm
        rot = Rotation2d(
            self.turnEncoder.get_absolute_position().value * math.tau
        )
        return SwerveModulePosition(distance, rot)

    def lock(self):
        self.driveMotor.set_control(
            DutyCycleOut(0, override_brake_dur_neutral=True)
        )
        self.turningMotor.set_control(
            DutyCycleOut(0, override_brake_dur_neutral=True)
        )

    def setDesiredState(
        self, desiredState: SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        encoderRotation = Rotation2d(
            self.turnEncoder.get_absolute_position().value
        )

        # Optimize the reference state to avoid spinning further than 90 deg
        state = SwerveModuleState.optimize(desiredState, encoderRotation)

        # Scale speed by cosine of angle error. If the error is 90 degrees
        # cos(90) = 0, and the wheels will not spin. If the error is 0 degrees,
        # cos(0) = 1, and the wheels will spin at full speed. Values between
        # 0 and 90 degrees will scale the speed accordingly.
        state.speed *= (state.angle - encoderRotation).cos()

        SmartDashboard.putNumber(f'{self.name} state speed', state.speed)
        SmartDashboard.putNumber(f'{self.name} Velocity', self.driveMotor.get_velocity().value)
        SmartDashboard.putNumber(f'{self.name} Velocity*enc', self.driveMotor.get_velocity().value * (2 * math.pi * kWheelRadius))

        # Calculate the drive output from the drive PID controller.
        driveOutput = state.speed / (2 * math.pi * kWheelRadius)

        # This come -0.5 to 0.5, so we must make it radians by scaling it up
        turn_pos = self.turnEncoder.get_absolute_position().value * math.tau

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            turn_pos, state.angle.radians()
        )

        # Now set the motor outputs where 0 is none and 1 is full
        SmartDashboard.putNumber(f'{self.name} driveOutput', driveOutput)
        self.driveMotor.set_control(VelocityDutyCycle(driveOutput))
        self.turningMotor.set_control(DutyCycleOut(turnOutput))
