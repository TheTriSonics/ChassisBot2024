
import commands2
from wpimath.filter import SlewRateLimiter
import subsystems.mechanical.drivetrain as dt
from wpilib import Joystick, SmartDashboard
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
    rot_limiter: SlewRateLimiter = SlewRateLimiter(8)

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
        return self.controller.getRawAxis(4)

    def execute(self) -> None:
        xspeed = self.xspeed_limiter.calculate(self.get_driver_x())*dt.kMaxSpeed
        yspeed = self.yspeed_limiter.calculate(self.get_driver_y())*dt.kMaxSpeed
        rot = self.rot_limiter.calculate(
            self.get_driver_rot()
        )*dt.kMaxAngularSpeed
        SmartDashboard.putNumber("X Speed", xspeed)
        SmartDashboard.putNumber("y Speed", yspeed)
        SmartDashboard.putNumber("Rot", rot)
        self.drivetrain.drive(xspeed, yspeed, rot)

    def isFinished(self) -> bool:
        return False
