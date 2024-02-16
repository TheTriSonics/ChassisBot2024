from commands2 import Subsystem
from wpilib import DigitalInput

class Photoeyes(Subsystem):
    def __init__(self):
        super().__init__()

    def get_photoeye(self, PhotoeyeID):
        # Get the value of the photoeye at the DIO ID
        # Inverted because the value is True when the photoeye is not blocked
        return not DigitalInput(PhotoeyeID.value).get()
