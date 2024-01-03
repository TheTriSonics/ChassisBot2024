from math import pi
MAX_ANGULAR_SPEED = 2*pi  # 1 rotation per second
MAX_SPEED: float = 120.0  # 10fpm seems safe for a chassis bot
WHEEL_RADIUS: float = 2  # inches; double check that this is correct on bot
DRIVE_GEAR_RATIO: float = 7.131
EFFECTIVE_RADIUS: float = WHEEL_RADIUS / DRIVE_GEAR_RATIO
DRIVE_RESOLUTION: int = 2048
TURN_RESOLUTION: int = 4096