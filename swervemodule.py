#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import DutyCycleOut
from wpilib import SmartDashboard
from phoenix6 import configs, signals, controls

kWheelRadius = 4.9
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        canCoderChannel: int,
        name: str,
    ) -> None:
        self.name = name
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        """
        # self.driveMotor = wpilib.PWMSparkMax(driveMotorChannel)
        # self.turningMotor = wpilib.PWMSparkMax(turningMotorChannel)
        
        self.driveMotor = TalonFX(driveMotorChannel)
        self.turningMotor = TalonFX(turningMotorChannel)
        
        driveConfigurator = self.driveMotor.configurator
        drive_motor_configs = configs.MotorOutputConfigs()

        turnConfigurator = self.turningMotor.configurator
        turn_motor_configs = configs.MotorOutputConfigs()

        turn_motor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        turn_motor_configs.neutral_mode = signals.NeutralModeValue.BRAKE

        driveConfigurator.apply(drive_motor_configs)
        turnConfigurator.apply(turn_motor_configs)
        """
        self.turningEncoder = wpilib.Encoder(
            turningEncoderChannelA, turningEncoderChannelB
        )
        self.turningEncoder = wpilib.Encoder(
            turningEncoderChannelA, turningEncoderChannelB
        )
        """
        self.turnEncoder = CANcoder(canCoderChannel)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(0.003, 0, 0)
        self.turningPIDController = wpimath.controller.PIDController(0.25, 0, 0)

        self.driveMotor.
        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        speed = self.driveMotor.get_rotor_velocity().value
        rot = wpimath.geometry.Rotation2d(self.driveMotor.get_rotor_position().value)
        return wpimath.kinematics.SwerveModuleState(speed, rot)


    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.:46
        
        """
        speed = self.driveMotor.get_rotor_velocity().value
        rot = wpimath.geometry.Rotation2d(self.driveMotor.get_rotor_position().value)
        return wpimath.kinematics.SwerveModulePosition(speed, rot)

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        # encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())
        encoderRotation = wpimath.geometry.Rotation2d(self.turnEncoder.get_absolute_position().value)

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveMotor.get_rotor_velocity().value, state.speed
        )

        # This come 0-1, not a degree or radian, so we must make it radians
        turn_pos = self.turnEncoder.get_absolute_position().value * 2*math.pi
        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            turn_pos, state.angle.radians()
        )
        err = self.turningPIDController.getPositionError()
        SmartDashboard.putNumber(f'{self.name} er', err)
        # Display the turning encoder value for this module
        SmartDashboard.putNumber(self.name, turn_pos)
        SmartDashboard.putNumber(self.name + ' out', driveOutput)

        self.driveMotor.set_control(DutyCycleOut(driveOutput))
        self.turningMotor.set_control(DutyCycleOut(turnOutput))
