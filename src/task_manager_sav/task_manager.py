from .thread_utils import ThreadWithExc

class Task(object):
    def __init__(self, function):
        self.function = function
        self.task = ThreadWithExc(target=self.wraped_function, args=())
        
        self.done = False

    def start(self):
        if not self.task.is_alive() and not self.done:
            self.task.start()
    
    def wraped_function(self):
        self.function()
        self.done = True

    def stop(self):
        if self.task.is_alive() or not self.done:
            self.task.raiseExc(Exception)
        
