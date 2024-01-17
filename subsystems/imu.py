import commands2
from typing import Optional, List
from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d

# What we're calling an IMU or Gyro is really more of an AHRS and what that
# is is pretty well defined by the Wikipedia page:
# https://en.wikipedia.org/wiki/Attitude_and_heading_reference_system

# Information on this specific Pigeon2.0 can be found here:
# https://store.ctr-electronics.com/pigeon-2/


class IMU(commands2.SubsystemBase):
    gyro: Pigeon2 = Pigeon2(30, 'canivore')
    initial_heading: Rotation2d
    xoffset: float = 0
    yoffset: float = 0

    def __init__(self):
        super().__init__()

    def reset(self) -> None:
        # TODO: no idea if this is right; check docs
        self.gyro.zeroGyroBiasNow()

    def get_yaw(self) -> Optional[float]:
        self.gyro.getYaw()

    def get_pitch(self) -> Optional[float]:
        self.gyro.getPitch()

    def get_roll(self) -> Optional[float]:
        return self.gyro.getRoll()

    def get_ypr(self) -> List[float]:
        _, ypr = self.gyro.getYawPitchRoll()
        # TODO: Check for error codes and do something?
        # This could be a case where we check for an exception and raise it,
        # thus halting the robot, but only if we're not attached to the FMS.
        # If we are, then perhaps return the last value and hope it resolves?
        return ypr

    def periodic(self) -> None:
        SmartDashboard.putNumber('Yaw', self.gyro.getYaw())
