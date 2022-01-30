from abc import ABC, abstractmethod

class World(ABC):
    ROBOT_RADIUS = 0.105 / 2.
    @abstractmethod
    def is_valid(self, pose):
        raise NotImplementedError

    @abstractmethod
    def get_dist(self, sources, angles):
        raise NotImplementedError

    @abstractmethod
    def sample(self, particles):
        raise NotImplementedError
