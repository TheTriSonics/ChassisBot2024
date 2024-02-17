import constants as ct
from commands2 import Subsystem
from rev import CANSparkMax

class ClimberSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        # Initialize the motor controller
        # Motor Type: 2x 775 Pro
        self.climber_motor_L = CANSparkMax(ct.CLIMBER_MOTOR_L)
        self.climber_motor_R = CANSparkMax(ct.CLIMBER_MOTOR_R)

        # Invert one motor so they both go the same direction
        self.climber_motor_R.setInverted(True)

    def go_up(self):
        # Set the motor to go up
        self.climber_motor_L.set(1.0)
        self.climber_motor_R.set(1.0)

    def go_down(self):
        # Set the motor to go down
        self.climber_motor_L.set(-1.0)
        self.climber_motor_R.set(-1.0)

    def stop(self):
        # Stop the motor
        self.climber_motor_L.set(0.0)
        self.climber_motor_R.set(0.0)

    def hold(self):
        # Hold the motor in place
        # TODO: Check to see if this is even necessary
        self.climber_motor_L.set(0.05)
        self.climber_motor_R.set(0.05)
