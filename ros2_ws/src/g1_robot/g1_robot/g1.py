from __future__ import annotations

import time
import threading
from g1_robot.robot_interface import RobotInterface, RobotConfig, RobotCommand, RobotMode

try:
    # Installed SDK exposes the class as `LocoClient` (not `G1LocoClient`).
    from unitree_sdk2py.core.channel import ChannelFactory
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
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
        self._client: LocoClient | None = None
        self._ready = False
        self._lock = threading.Lock()
        # What the last send_command() actually handed to the SDK, and the SDK's
        # return value — read by robot_node for the /g1/robot_cmd trial log.
        # Bookkeeping only; nothing here changes what is sent.
        self.last_sent: tuple[float, float, float] | None = None
        self.last_ret = None

    def setup(self, config: RobotConfig) -> bool:
        self._config = config

        if not _SDK_AVAILABLE:
            print("[G1] Unitree SDK not found — running in stub mode.")
            self._ready = True
            return True

        try:
            # 2nd arg is the host NIC name (e.g. "enp4s0"), NOT the robot IP.
            # Empty string → let the SDK auto-detect the interface.
            if config.net_iface:
                ChannelFactory().Init(0, config.net_iface)
            else:
                ChannelFactory().Init(0)
            self._client = LocoClient()
            # Was 10 s: a call that got no reply froze robot_node for 10 s, during
            # which no STOP could be sent (trials 20261027_173238, _180825). A
            # normal reply takes ~2 ms.
            self._client.SetTimeout(config.sdk_timeout_s)
            self._client.Init()
            time.sleep(1.0)  # give SDK time to establish DDS connection

            # Proven sequence (matches g1_loco_client_example): after Init() the
            # robot must be STANDING before Move() walks. We do NOT auto-StandUp
            # here — the robot should already be standing (stand it via the
            # remote / example menu, or call stand()). This avoids the node
            # unexpectedly raising the robot from a sitting/damped pose on launch.
            self._ready = True
            print(f"[G1] Connected via DDS (iface='{config.net_iface or 'auto'}'). "
                  f"Ensure the robot is STANDING before sending /cmd_vel.")
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

        # SetVelocity is what StopMove() and Move() call internally, with the
        # same arguments, but it also returns the robot's reply code (0 = OK)
        # where they discard it.
        with self._lock:
            self.last_sent = None
            self.last_ret = None
            try:
                if cmd.mode == RobotMode.STOP:
                    self.last_ret = self._client.SetVelocity(0.0, 0.0, 0.0)
                else:
                    vx  = float(max(-self.MAX_VX,   min(self.MAX_VX,   cmd.vx)))
                    vy  = float(max(-self.MAX_VY,   min(self.MAX_VY,   cmd.vy)))
                    wz  = float(max(-self.MAX_VYAW, min(self.MAX_VYAW, cmd.wz)))
                    self.last_ret = self._client.SetVelocity(vx, vy, wz, 1)  # = Move()
                    self.last_sent = (vx, vy, wz)
                return self.last_ret == 0
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
                self._client.StandUp()  # FSM 4 — stand, ready to accept Move()
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
