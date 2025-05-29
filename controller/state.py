from abc import abstractmethod, ABCMeta


class State(metaclass=ABCMeta):
    @abstractmethod
    def serialize(self):
        pass

    @abstractmethod
    def deserialize(self, raw_state):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def after_update(self):
        pass

    @abstractmethod
    def reset(self):
        pass
