from __future__ import annotations

from g1_robot.robot_interface import RobotInterface, RobotConfig, RobotCommand, RobotMode

# TODO: install Unitree G1 SDK and replace stubs
# SDK repo: https://github.com/unitreerobotics/unitree_sdk2_python
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeCmd_
    _SDK_AVAILABLE = True
except ImportError:
    _SDK_AVAILABLE = False


class G1Robot(RobotInterface):
    def __init__(self):
        self._config: RobotConfig | None = None
        self._publisher = None

    def setup(self, config: RobotConfig) -> bool:
        self._config = config

        if not _SDK_AVAILABLE:
            print("[G1] Unitree SDK not installed — running in stub mode.")
            return True

        self._publisher = ChannelPublisher("rt/sportmodestate", SportModeCmd_)
        self._publisher.Init()
        print(f"[G1] Connected to {config.ip}:{config.port}")
        return True

    def send_command(self, cmd: RobotCommand) -> bool:
        if not _SDK_AVAILABLE or self._publisher is None:
            print(f"[G1 STUB] cmd={cmd.mode.name} vx={cmd.vx:.2f} vy={cmd.vy:.2f} wz={cmd.wz:.2f}")
            return True

        msg = SportModeCmd_()
        msg.vx = cmd.vx
        msg.vy = cmd.vy
        msg.vyaw = cmd.wz
        self._publisher.Write(msg)
        return True

    def stop(self) -> bool:
        return self.send_command(RobotCommand(mode=RobotMode.STOP))
