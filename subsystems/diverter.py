from wpilib import CANSparkMax, SubsystemBase
from enum import Enum

class Diverter(SubsystemBase):
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


    def go_direction(self, direction):
        self.motor.set(0.5)  # Set motor speed to 50% power
        self.direction = direction
        self.motor.set(direction.value)

    def go_shooter(self):
        self.set_direction(Diverter.Direction.SHOOTER)

    def go_amp(self):
        self.set_direction(Diverter.Direction.AMP)

    def halt(self):
        self.motor.stopMotor()