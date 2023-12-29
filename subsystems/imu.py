import commands2
from typing import Optional
from ctre.sensors import Pigeon2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d


class IMU(commands2.SubsystemBase):
    gyro: Pigeon2
    initial_heading: Rotation2d
    xoffset: float = 0
    yoffset: float = 0

    def __init__(self):
        super().__init__()

    def reset(self) -> None:
        self.gyro.zeroGyroBiasNow()  # no idea if this is right; check docs

    def get_yaw(self) -> Optional[float]:
        self.gyro.getYaw()

    def get_pitch(self) -> Optional[float]:
        self.gyro.getPitch()

    def get_roll(self) -> Optional[float]:
        return self.gyro.getRoll()

    def get_ypr(self):
        return self.gyro.getYawPitchRoll()

    def periodic(self) -> None:
        SmartDashboard.putNumber('Yaw', self.gyro.getYaw())
