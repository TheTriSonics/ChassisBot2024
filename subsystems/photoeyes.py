from commands2 import Subsystem


class PhotoEyeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

    def intake_full(self) -> bool:
        # Implement the logic for intake full here
        print("Intake pull method called")
        return False

    def shooter_loaded(self) -> bool:
        # Implement the logic for shooter loaded here
        print("Shooter loaded method called")
        return False

    def amp_loaded(self) -> bool:
        # Implement the logic for amp loaded here
        print("Amp loaded method called")
        return False