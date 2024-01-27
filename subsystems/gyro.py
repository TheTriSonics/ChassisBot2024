from phoenix6.hardware import Pigeon2

class Gyro():
    def __init__ (self):
        self.gyro = Pigeon2(41)


    def get_yaw(self) -> float:
        return self.gyro.get_yaw().value
    
    def set_yaw(self, x):
        self.gyro.set_yaw(x)
        