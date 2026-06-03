from __future__ import annotations

import time
import threading
from g1_robot.robot_interface import RobotInterface, RobotConfig, RobotCommand, RobotMode

try:
    from unitree_sdk2py.core.channel import ChannelFactory
    from unitree_sdk2py.g1.loco.g1_loco_client import G1LocoClient
    _SDK_AVAILABLE = True
except ImportError:
    _SDK_AVAILABLE = False


class G1Robot(RobotInterface):
    # G1 physical limits
    MAX_VX    =  0.8   # m/s forward
    MAX_VY    =  0.4   # m/s lateral
    MAX_VYAW  =  0.8   # rad/s yaw rate

    def __init__(self):
        self._config: RobotConfig | None = None
        self._client: G1LocoClient | None = None
        self._ready = False
        self._lock = threading.Lock()

    def setup(self, config: RobotConfig) -> bool:
        self._config = config

        if not _SDK_AVAILABLE:
            print("[G1] Unitree SDK not found — running in stub mode.")
            self._ready = True
            return True

        try:
            ChannelFactory.Instance().Init(0, config.ip)
            self._client = G1LocoClient()
            self._client.SetTimeout(10.0)
            self._client.Init()
            time.sleep(1.0)  # give SDK time to establish DDS connection

            # Enter balance-stand mode before accepting velocity commands
            self._client.BalanceStand()
            time.sleep(2.0)

            self._ready = True
            print(f"[G1] Connected and standing at {config.ip}")
            return True
        except Exception as e:
            print(f"[G1] Setup failed: {e}")
            return False

    def send_command(self, cmd: RobotCommand) -> bool:
        if not self._ready:
            return False

        if not _SDK_AVAILABLE or self._client is None:
            print(f"[G1 STUB] {cmd.mode.name}  vx={cmd.vx:.2f}  vy={cmd.vy:.2f}  wz={cmd.wz:.2f}")
            return True

        with self._lock:
            try:
                if cmd.mode == RobotMode.STOP:
                    self._client.StopMove()
                else:
                    vx  = float(max(-self.MAX_VX,   min(self.MAX_VX,   cmd.vx)))
                    vy  = float(max(-self.MAX_VY,   min(self.MAX_VY,   cmd.vy)))
                    wz  = float(max(-self.MAX_VYAW, min(self.MAX_VYAW, cmd.wz)))
                    self._client.Move(vx, vy, wz)
                return True
            except Exception as e:
                print(f"[G1] send_command failed: {e}")
                return False

    def stop(self) -> bool:
        return self.send_command(RobotCommand(mode=RobotMode.STOP))

    def stand(self) -> bool:
        if not _SDK_AVAILABLE or self._client is None:
            print("[G1 STUB] stand")
            return True
        with self._lock:
            try:
                self._client.BalanceStand()
                return True
            except Exception as e:
                print(f"[G1] stand failed: {e}")
                return False

    def shutdown(self) -> None:
        self.stop()
        if _SDK_AVAILABLE and self._client is not None:
            with self._lock:
                try:
                    self._client.StandUp()  # return to upright rest pose
                except Exception:
                    pass
