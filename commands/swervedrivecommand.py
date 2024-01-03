
import commands2
from wpimath.filter import SlewRateLimiter
import constants
import subsystems.drivetrain as dt
from wpilib import Joystick
from functools import wraps


def square(func):
    @wraps(func)
    def wrapper():
        x = func()
        sign = 1 if x > 0 else -1
        return x*x*sign
    return wrapper


def deadband(val):
    def actual_dec(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            x = func(*args, **kwargs)
            if abs(x) >= val:
                return x
            return 0
        return wrapper
    return actual_dec


class SwerveDriveCommand(commands2.CommandBase):
    xspeed_limiter: SlewRateLimiter = SlewRateLimiter(1.5)
    yspeed_limiter: SlewRateLimiter = SlewRateLimiter(1.5)
    rot_limiter: SlewRateLimiter = SlewRateLimiter(3)

    def __init__(self, drive, controller: Joystick):
        super().__init__()
        self.drivetrain = drive
        self.controller = controller
        self.addRequirements(
            self.drivetrain,
        )

    def initialize(self) -> None:
        pass

 
    def get_driver_x(self) -> float:
        return self.controller.getRawAxis(0)

   
    def get_driver_y(self) -> float:
        return self.controller.getRawAxis(1)

    def get_driver_rot(self) -> float:
        return self.controller.getRawAxis(5)

    def execute(self) -> None:
        xspeed = self.xspeed_limiter.calculate(self.get_driver_x())*constants.MAX_SPEED
        yspeed = self.yspeed_limiter.calculate(self.get_driver_y())*constants.MAX_SPEED
        rot = self.rot_limiter.calculate(
            self.get_driver_rot()
        )*constants.MAX_ANGULAR_SPEED*-0.5
        self.drivetrain.drive(xspeed, yspeed, rot)

    def isFinished(self) -> bool:
        return False
