import constants as ct
from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from rev import CANSparkMax


class Shooter(Subsystem):
    def __init__(self):
        super().__init__()

        # Motor Types: 2x Krakens for shooter, 1x Kraken for feeder, 2x NEO 550s for tilt
        self.shooter_motor_L = TalonFX(ct.SHOOTER_MOTOR_L)
        self.shooter_motor_R = TalonFX(ct.SHOOTER_MOTOR_R)
        self.feeder_motor = TalonFX(ct.SHOOTER_FEEDER_MOTOR)

        self.tilt_motor_L = CANSparkMax(ct.SHOOTER_TILT_MOTOR_L)
        self.tilt_motor_R = CANSparkMax(ct.SHOOTER_TILT_MOTOR_R)

        self.tilt_encoder = CANcoder(ct.SHOOTER_TILT_ENCODER)

        self.tilt_motor_R.setInverted(True)

        # Initialize the target speed
        self.target_speed = 0

    def set_angle(self, value):
        # Set the angle of the shooter
        # self.elevation_motor.set(value)
        pass

    def set_speed(self, value):
        # Set the target speed of the shooter motor
        # self.target_speed = value
        pass

    def is_up_to_speed(self):
        # Check if the shooter motor is up to speed
        # return self.encoder.getRate() >= self.target_speed
        pass

    def reverse(self):
        # Reverse the direction of the shooter motor
        # self.shooter_motor.set(-self.target_speed)
        pass

    def halt(self):
        # Stop the shooter motor
        # self.shooter_motor.set(0)
        pass
