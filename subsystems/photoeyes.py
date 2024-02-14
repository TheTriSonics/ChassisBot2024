import wpilib

class PhotoEyeSubsystem(wpilib.Subsystem):
    def __init__(self):
        super().__init__("PhotoEyeSubsystem")

    def intake_pull(self):
        # Implement the logic for intake pull here
        print("Intake pull method called")

    def shooter_loaded(self):
        # Implement the logic for shooter loaded here
        print("Shooter loaded method called")

    def amp_loaded(self):
        # Implement the logic for amp loaded here
        print("Amp loaded method called")