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
    # Host network interface on the robot's subnet (e.g. "enp4s0"). DDS binds to
    # this NIC to discover the G1; it is NOT the robot's IP. Empty = auto-detect.
    net_iface: str = ""
    # How long one SDK call may wait for the robot's reply. The SDK waits this
    # long twice (send, then reply), so a call that gets no reply returns after
    # at most ~2x this, with code 3104 (RPC_ERR_CLIENT_API_TIMEOUT).
    sdk_timeout_s: float = 0.25


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
