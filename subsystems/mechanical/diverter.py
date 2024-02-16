from enum import Enum
from commands2 import Subsystem
from rev import CANSparkMax

#TODO: This doesn't fully exist anymore. Diverter wheel is now part of the Amp System.
class Diverter(Subsystem):
    class Direction(Enum):
        SHOOTER = 1
        AMP = -1

    def __init__(self, motor_id):
        super().__init__()
        self.motor = CANSparkMax(motor_id, CANSparkMax.MotorType.kBrushless)
        self.direction = None

    def set_amp(self):
        self.direction = Diverter.Direction.AMP

    def set_shooter(self):
        self.direction = Diverter.Direction.SHOOTER

    def set_direction(self, direction):
        self.motor.set(0.5 * direction)  # Set motor speed to 50% power

    def go_shooter(self):
        self.set_direction(Diverter.Direction.SHOOTER)

    def go_amp(self):
        self.set_direction(Diverter.Direction.AMP)

    def halt(self):
        self.motor.stopMotor()
