from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum


class RobotMode(Enum):
    STAND  = 0
    WALK   = 1
    STOP   = 2


@dataclass
class RobotConfig:
    ip: str
    port: int = 8080


@dataclass
class RobotCommand:
    mode: RobotMode
    vx: float = 0.0   # forward velocity  (m/s)
    vy: float = 0.0   # lateral velocity  (m/s)
    wz: float = 0.0   # yaw rate          (rad/s)


class RobotInterface(ABC):
    @abstractmethod
    def setup(self, config: RobotConfig) -> bool: ...

    @abstractmethod
    def send_command(self, cmd: RobotCommand) -> bool: ...

    @abstractmethod
    def stop(self) -> bool: ...
