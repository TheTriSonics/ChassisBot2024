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

        # Initialize the motor controllers
        # Replace with the type of motor controller you're using
        self.lift_motor = TalonFX(0)
        self.roller_motor = CANSparkMax(1)

    def set_eleveation(self, Elevation):
        # Set the elevation of the lift
        pass

    def set_roller_speed(self, value):
        # Set the speed of the roller motor
        #self.roller_motor.set(value)
        pass