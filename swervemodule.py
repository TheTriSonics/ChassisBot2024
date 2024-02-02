#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpimath.kinematics
import wpimath.trajectory

from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import DutyCycleOut
from phoenix6 import configs, signals
from wpilib import SmartDashboard


kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau
kWheelRadius = 4.9  / 100 # m
kGearRatio = 7.131
kEncoderResolution = 2048

encoder_to_mech_ratio = 4.6 / 100.0


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        canCoderChannel: int,
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

        drive_feedback = configs.FeedbackConfigs()
        drive_feedback.with_sensor_to_mechanism_ratio(1)
        driveConfigurator.apply(drive_feedback)

        turnConfigurator = self.turningMotor.configurator
        turn_motor_configs = configs.MotorOutputConfigs()
        turn_motor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        turn_motor_configs.neutral_mode = signals.NeutralModeValue.BRAKE
        turnConfigurator.apply(turn_motor_configs)

        self.turnEncoder = CANcoder(canCoderChannel)

        self.drivePIDController = PIDController(0.1, 0, 0)
        self.turningPIDController = PIDController(0.25, 0, 0)

        # Limit the PID Controller's input range between -pi and pi and
        # set the input to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)
        self.driveMotor.set_position(0)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        speed = (
            self.driveMotor.get_rotor_velocity().value * encoder_to_mech_ratio
        )
        SmartDashboard.putNumber(f'{self.name} speed', speed)
        rot = Rotation2d(
            self.turnEncoder.get_absolute_position().value * math.tau
        )
        return wpimath.kinematics.SwerveModuleState(speed, rot)

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.:46


        """
        pos = self.driveMotor.get_position().value
        # Now convert from rotation units to centimeters
        # distance = pos * math.tau * kWheelRadius
        distance = pos * encoder_to_mech_ratio  # convert rotations to cm
        rot = Rotation2d(
            self.turnEncoder.get_absolute_position().value * math.tau
        )
        return wpimath.kinematics.SwerveModulePosition(distance, rot)

    def lock(self):
        self.driveMotor.set_control(
            DutyCycleOut(0, override_brake_dur_neutral=True)
        )
        self.turningMotor.set_control(
            DutyCycleOut(0, override_brake_dur_neutral=True)
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        encoderRotation = Rotation2d(
            self.turnEncoder.get_absolute_position().value
        )

        # Optimize the reference state to avoid spinning further than 90 deg
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. If the error is 90 degrees
        # cos(90) = 0, and the wheels will not spin. If the error is 0 degrees,
        # cos(0) = 1, and the wheels will spin at full speed. Values between
        # 0 and 90 degrees will scale the speed accordingly.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveMotor.get_rotor_velocity().value * encoder_to_mech_ratio, state.speed
        )

        # This come -0.5 to 0.5, so we must make it radians by scaling it up
        turn_pos = self.turnEncoder.get_absolute_position().value * math.tau

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            turn_pos, state.angle.radians()
        )

        # Now set the motor outputs where 0 is none and 1 is full
        self.driveMotor.set_control(DutyCycleOut(driveOutput))
        self.turningMotor.set_control(DutyCycleOut(turnOutput))
