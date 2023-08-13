from .task import *

class RobotTaskScheduler:

    def __init__(self) -> None:
        self.tasks = list()

    def add(self, task: RobotTask) -> None:
        self.tasks.append(task)

    def next(self):
        self.tasks.pop(0)
    
    def empty(self) -> bool:
        return (len(self.tasks)== 0)
    
    def current_task(self) -> RobotTask:
        if not self.empty():
            return self.tasks[0]
        else:
            return None
        
    def spin_once(self, time) -> None:
        if not self.empty():
            if self.current_task().begin_time is None:
                self.current_task().begin_time = float(time)
            self.current_task().check_completion()
            if self.current_task().is_done():
                print(f'{self.current_task().name} is done')
                self.next()
            else:
                print(f'current task: {self.current_task().name}')
                self.current_task().actuate()