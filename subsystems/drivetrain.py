import commands2
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6 import configs, signals, controls
from commands.swervedrivecommand import SwerveDriveCommand
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.controller import PIDController
from wpilib import Notifier, SmartDashboard, Joystick, AnalogInput
from wpimath.kinematics import (
    SwerveModulePosition,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    ChassisSpeeds,
)
from math import pi


MAX_ANGULAR_SPEED = 2*pi  # 1 rotation per second
MAX_SPEED: float = 1.0  # 10fpm seems safe for a chassis bot
WHEEL_RADIUS: float = 2  # inches; double check that this is correct on bot
DRIVE_GEAR_RATIO: float = 7.131
EFFECTIVE_RADIUS: float = WHEEL_RADIUS / DRIVE_GEAR_RATIO
DRIVE_RESOLUTION: int = 2048
TURN_RESOLUTION: int = 1


class SwerveModule():

    drive_motor: TalonFX
    turn_motor: TalonFX
    turn_encoder: CANcoder
    encoder_offset: float = 0
    turn_pid_controller: PIDController = PIDController(1/1000, 0, 0)

    drive_disabled = False
    name = ""

    def __init__(self, drive_motor_can_id, turn_motor_can_id,
                 turn_encoder_can_id, turn_encoder_offset=0,
                 name='', drive_disabled=False):
        self.drive_motor = TalonFX(drive_motor_can_id)
        self.turn_motor = TalonFX(turn_motor_can_id)
        self.turn_encoder = CANcoder(turn_encoder_can_id, "rio")
        self.turn_encoder_offset = turn_encoder_offset
        self.name = name
        self.drive_disabled = drive_disabled

        driveConfigurator = self.drive_motor.configurator
        drive_motor_configs = configs.MotorOutputConfigs()

        turnConfigurator = self.turn_motor.configurator
        turn_motor_configs = configs.MotorOutputConfigs()

        drive_motor_configs.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        drive_motor_configs.neutral_mode = signals.NeutralModeValue.BRAKE

        slot0Turning = configs.Slot0Configs()
        slot0Driving = configs.Slot0Configs()

        slot0Turning.k_p = 0.1
        slot0Turning.k_d = 0.002

        slot0Driving.k_v = 0.0465
        slot0Driving.k_p = 0.0

        driveConfigurator.apply(drive_motor_configs)
        driveConfigurator.apply(slot0Driving)

        turnConfigurator.apply(slot0Turning)

    def get_drive_velocity(self) -> float:
        vel = self.drive_motor.get_velocity().value
        return vel * 10/DRIVE_RESOLUTION * 2 * pi * EFFECTIVE_RADIUS

    def get_turn_position_radians(self) -> float:
        return self.get_turn_position()/TURN_RESOLUTION * 2 * pi

    def get_raw_turn_position(self) -> float:
        return self.turn_encoder.get_absolute_position().value

    def get_turn_position(self) -> float:
        return self.turn_encoder.get_absolute_position().value

    def get_state(self) -> SwerveModulePosition:
        distance = (
            self.drive_motor.get_rotor_position().value / 
            DRIVE_RESOLUTION*2*pi*EFFECTIVE_RADIUS
        )
        SmartDashboard.putNumber(f'd {self.name}', distance)
        rads = self.get_turn_position_radians()
        return SwerveModulePosition(distance, Rotation2d(rads))

    def set_state(self, desired_state: SwerveModuleState) -> None:
        curr_pos: float = self.get_turn_position()
        turn_angle: float = 2*pi/TURN_RESOLUTION*curr_pos
        state: SwerveModuleState = SwerveModuleState.optimize(
            desired_state, Rotation2d(turn_angle)
        )
        drive_vel: float = (
            state.speed / (2*pi*EFFECTIVE_RADIUS)
            * DRIVE_RESOLUTION * 0.1
        )
        set_pos: float = state.angle.radians()/(2*pi) * TURN_RESOLUTION
        revs: int = round(curr_pos / TURN_RESOLUTION)
        set_pos += revs * TURN_RESOLUTION
        while set_pos > curr_pos + TURN_RESOLUTION / 2:
            set_pos -= TURN_RESOLUTION
        while set_pos < curr_pos - TURN_RESOLUTION / 2:
            set_pos += TURN_RESOLUTION

        if not self.drive_disabled:
            SmartDashboard.putNumber(self.name + ' drive vel', drive_vel)
            # This dutyCycleOut wants a value between -1.0 and 1.0 for speed
            # self.drive_motor.set_control(controls.DutyCycleOut(drive_vel))
            rotations_per_second = drive_vel / (2*pi*EFFECTIVE_RADIUS)
            c = controls.VelocityDutyCycle(velocity=rotations_per_second)
            self.drive_motor.set_control(c)
        self.turn_pid_controller.setSetpoint(set_pos)
        turn_power: float = self.turn_pid_controller.calculate(curr_pos)
        self.turn_motor.set_control(controls.DutyCycleOut(turn_power))

    def reset_turn_encoder(self) -> None:
        self.encoder_offset = self.turn_encoder.get_position().value

    def stopDriveMotor(self) -> None:
        self.drive_motor.set_control(controls.DutyCycleOut(0))
        self.turn_motor.set_control(controls.DutyCycleOut(0))


class SwerveDrivetrain(commands2.SubsystemBase):
    notifier: Notifier
    kMaxSpeed: float = 15.6 * 12  # inches per second
    kMaxAngularSpeed: float = 2*pi  # rotations per second in radians
    halfWheelBase = 23.251 / 2.0
    halfTrackWidth = 22.25 / 2.0
    front_left_location = Translation2d(halfWheelBase, halfTrackWidth)
    front_right_location = Translation2d(halfWheelBase, -halfTrackWidth)
    back_left_location = Translation2d(-halfWheelBase, halfTrackWidth)
    back_right_location = Translation2d(-halfWheelBase, -halfWheelBase)
    # Create swerve modules
    front_left = SwerveModule(12, 22, 32, 0, "Front Left", False)
    front_right = SwerveModule(11, 21, 31, 0, "Back Left", False)
    back_left = SwerveModule(14, 24, 34, 0, "Back Right", False)
    back_right = SwerveModule(13, 23, 33, 0, "Front Right", False)
    field_relative = False
    drive_aligned = False
    goalX = 12
    goalY = 24
    kP = 0.015
    default_pos = (
        SwerveModulePosition(0, Rotation2d(0)),
        SwerveModulePosition(0, Rotation2d(0)),
        SwerveModulePosition(0, Rotation2d(0)),
        SwerveModulePosition(0, Rotation2d(0)),
    )

    kinematics = SwerveDrive4Kinematics(
        front_left_location,
        front_right_location,
        back_left_location,
        back_right_location,
    )

    odometry = SwerveDrive4Odometry(kinematics, Rotation2d(0),
                                    default_pos)

    def __init__(self, imu, joystick) -> None:
        self.upd_counter = 0
        super().__init__()
        self.notifier = Notifier(self.update_odometry)
        self.notifier.startPeriodic(0.01)

        self.imu = imu

        self.odometry.resetPosition(
            gyroAngle=Rotation2d(0),
            pose=Pose2d(),
            modulePositions=(
             self.front_left.get_state(),
             self.front_right.get_state(),
             self.back_left.get_state(),
             self.back_right.get_state())
        )
        # self.odometry.resetPosition(Rotation2d(),tuple[],Pose2d())
        self.setDefaultCommand(SwerveDriveCommand(self, joystick))

    def get_heading(self) -> float:
        y = self.imu.get_yaw()
        return 0.0 if y is None else y

    # Helper method to return the heading as a Rotation2d typed object because
    # that's often how we need it.
    def get_heading_rotation_2d(self) -> Rotation2d:
        return Rotation2d(self.get_heading())

    def update_odometry(self) -> None:
        self.upd_counter += 1
        SmartDashboard.putNumber('upd counter', self.upd_counter)
        self.odometry.update(
            self.get_heading_rotation_2d(),
            (self.front_left.get_state(),
             self.front_right.get_state(),
             self.back_left.get_state(),
             self.back_right.get_state())
        )

    def drive(self, xspeed: float, yspeed: float, rot: float):
        # if self.drive_aligned:
        if False:
            #  TOOD: (or not) Drive to the limelight target
            pass
        else:
            from math import atan2, pi
            current_pose = self.odometry.getPose()
            deltaX = self.goalX - current_pose.X()
            deltaY = self.goalY - current_pose.Y()

            SmartDashboard.putNumber('deltaX', deltaX)
            SmartDashboard.putNumber('deltaY', deltaY)

            target_angle = atan2(deltaY, deltaX)*180/pi
            current_angle = current_pose.rotation().degrees()
            SmartDashboard.putNumber('target', target_angle)
            SmartDashboard.putNumber('curr', current_angle)
            error_angle = (target_angle - current_angle)
            SmartDashboard.putNumber('erro1', error_angle)
            while error_angle < -180:
                error_angle += 360
            while error_angle > 180:
                error_angle += 360

            SmartDashboard.putNumber('erro2', error_angle)
            rot = error_angle / 180 * MAX_ANGULAR_SPEED
            SmartDashboard.putNumber('rot drivetrain', rot)

        if self.field_relative:
            fl, fr, bl, br = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xspeed, yspeed, rot, self.get_heading_rotation_2d()
                )
            )
        else:
            # Wants inputs in meters per second, not raw stick value
            cs = ChassisSpeeds(xspeed, yspeed, rot)
            fl, fr, bl, br = (
                self.kinematics.toSwerveModuleStates(cs)
            )
        SmartDashboard.putNumber("FL Speed", fl.speed)
        SmartDashboard.putNumber("FR Speed", fr.speed)
        SmartDashboard.putNumber("BL Speed", bl.speed)
        SmartDashboard.putNumber("BR Speed", br.speed)

        self.front_left.set_state(fl)
        self.front_right.set_state(fr)
        self.back_left.set_state(bl)
        self.back_right.set_state(br)

    def set_odometry_to_pose(self, pose: Pose2d) -> None:
        # TODO: Implement?
        pass

    def set_odometry(self, x: float, y: float, heading: float) -> None:
        # broken
        pass

    def _get_modules(self) -> list[SwerveModule]:
        return [self.front_left, self.front_right,
                self.back_left, self.back_right]

    # def force_brake_mode(self) -> None:
    #     for module in self._get_modules():
    #         module.drive_motor.setNeutralMode(NeutralMode.Brake)

    # def force_coast_mode(self) -> None:
    #     for module in self._get_modules():
    #         module.drive_motor.setNeutralMode(NeutralMode.Coast)

    def reset_turn_encoders(self):
        for module in self._get_modules():
            module.reset_turn_encoder()

    def stop_drive_motor(self):
        for module in self._get_modules():
            module.stopDriveMotor()

    # This method will be called once per scheduler run and should not
    # be called directly.
    def periodic(self):
        SmartDashboard.putNumber("Back Right",
                                 self.back_right.get_raw_turn_position())
        SmartDashboard.putNumber("Back Left",
                                 self.back_left.get_raw_turn_position())
        SmartDashboard.putNumber("Front Right",
                                 self.front_right.get_raw_turn_position())
        SmartDashboard.putNumber("Front Left",
                                 self.front_left.get_raw_turn_position())
        """
        pose: Pose2d = self.get_odometry().getPoseMeters()
        SmartDashboard.putNumber("X", pose.X())
        SmartDashboard.putNumber("Y", pose.Y())
        SmartDashboard.putNumber("Rotation", pose.rotation().degrees())
        """
        pass
