from .model import *

class RobotTask:

    def __init__(self, name) -> None:
        self.done = False
        self.name = name
        self.check_completion_method = None
        self.actuate_method = None
        self.robot: RobotModel = None
        self.target = None
        self.begin_time = None

    def set_check_completion_method(self, func):
        self.check_completion_method = func
    
    def set_actuate_method(self, func):
        self.actuate_method = func

    def is_done(self) -> bool:
        return self.done
    
    def check_completion(self):
        if self.check_completion_method is not None:
            self.check_completion_method(self)

    def actuate(self):
        if self.actuate_method is not None:
            self.actuate_method(self)