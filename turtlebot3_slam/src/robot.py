from abc import ABCMeta, abstractmethod
class Robot:
    __metaclass__ = ABCMeta
    @abstractmethod
    def sensor_model(self, **kwargs):
        raise NotImplementedError
    
    @abstractmethod
    def motionModel(self, **kwargs):
        raise NotImplementedError

class TurtleBot(Robot):
    def sensor_model(self, **kwargs):
        pass
    def motionModel(self, **kwargs):
        pass