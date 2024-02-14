from commands2 import Subsystem
from phoenix6.hardware import TalonFX


class Shooter(Subsystem):
    def __init__(self):
        super().__init__()

        # Initialize the motor controllers
        # Replace with the type of motor controller you're using
        self.shooter_motor = TalonFX(0)
        self.elevation_motor = TalonFX(1)

        # Initialize the target speed
        self.target_speed = 0

    def set_elevation(self, value):
        # Set the speed of the elevation motor
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
