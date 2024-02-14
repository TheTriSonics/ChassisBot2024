import wpilib
from ctre import WPI_TalonSRX

class ClimberSubsystem(wpilib.command.Subsystem):
    def __init__(self):
        super().__init__("ClimberSubsystem")

        # Initialize the motor controller
        self.climber_motor = WPI_TalonSRX(1)

    def go_up(self):
        # Set the motor to go up
        self.climber_motor.set(1.0)

    def go_down(self):
        # Set the motor to go down
        self.climber_motor.set(-1.0)

    def stop(self):
        # Stop the motor
        self.climber_motor.set(0.0)