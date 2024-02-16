from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from rev import CANSparkMax


class Intake(Subsystem):
    class Tilt(Enum):
        UP = -1
        DOWN = 1

    class Direction(Enum):
        IN = 1
        OUT = -1
        STOP = 0

    def __init__(self):
        super().__init__()

        # Intake tilt is to tilt intake up and down, has Top/Bottom limit switches. Snowblower motor
        #Roller motors are for the top and bottom rollers. 2x Bag Motors
        self.intake_tilt_motor = CANSparkMax(0)
        self.roller_motors = CANSparkMax(1)

    def tilt(self, Tilt):
        # Set the tilt of the intake
        pass

    def set_roller_speed(self, Direction):
        # Set the speed of the roller motors
        self.roller_motors.set(Direction)
        pass