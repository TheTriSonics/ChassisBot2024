import constants as ct
from phoenix6.hardware import Pigeon2
from commands2 import Subsystem


class Gyro(Subsystem):
    def __init__(self):
        super().__init__()
        self.gyro = Pigeon2(ct.PIGEON)

    def get_yaw(self) -> float:
        return self.gyro.get_yaw().value

    def set_yaw(self, x):
        self.gyro.set_yaw(x)
