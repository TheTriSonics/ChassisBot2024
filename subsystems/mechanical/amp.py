import constants as ct
from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from rev import CANSparkMax


class Amp(Subsystem):
    class Elevation(Enum):
        HOME = 0
        AMP = 1
        TRAP = 2

    def __init__(self):
        super().__init__()

        # Motor types: 1x Kraken, 1x Bag Motor
        self.lift_motor = TalonFX(ct.AMP_LIFT_MOTOR)
        self.roller_motor = CANSparkMax(ct.AMP_ROLLER_MOTOR)

    def set_eleveation(self, Elevation):
        # Set the elevation of the lift
        pass

    def set_roller_speed(self, value):
        # Set the speed of the roller motor
        #self.roller_motor.set(value)
        pass