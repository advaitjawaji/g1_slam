"""
RL Locomotion Policy wrapper for Unitree G1.

Loads a pre-trained policy checkpoint from unitree_rl_gym / Isaac Lab and
provides a clean interface: observations in → joint targets out.

Checkpoint source:
    https://github.com/unitreerobotics/unitree_rl_gym
    Look for: g1/policy.pt or g1_23dof_*.pt

Policy architecture (standard legged_gym format):
    MLP: obs_dim → [512, 256, 128] → action_dim
    Observation: ~47 dims (legs + waist only, arms held fixed)
    Action: 12 dims (leg joints) or 14 dims (legs + waist)
"""
from __future__ import annotations

import math
import numpy as np

try:
    import torch
    import torch.nn as nn
    _TORCH_AVAILABLE = True
except ImportError:
    _TORCH_AVAILABLE = False


# ── Joint ordering (matches unitree_rl_gym G1 locomotion policy) ──────────
# Policy controls LEGS + WAIST only. Arms held at default.
POLICY_JOINT_NAMES = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
    "waist_yaw_link",
    "waist_roll_link",
]
NUM_POLICY_JOINTS = len(POLICY_JOINT_NAMES)  # 14

# Default standing joint angles (radians) — tuned for G1
DEFAULT_JOINT_POS = np.array([
    # left leg: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
    -0.10, 0.0, 0.0, 0.30, -0.20, 0.0,
    # right leg
    -0.10, 0.0, 0.0, 0.30, -0.20, 0.0,
    # waist: yaw, roll
    0.0, 0.0,
], dtype=np.float32)

# PD gains matching Unitree hardware
KP = np.array([
    100, 80, 60, 150, 40, 40,   # left leg
    100, 80, 60, 150, 40, 40,   # right leg
    200, 80,                     # waist
], dtype=np.float32)

KD = np.array([
    5, 4, 3, 6, 2, 2,   # left leg
    5, 4, 3, 6, 2, 2,   # right leg
    8, 4,                # waist
], dtype=np.float32)

# Observation scaling (matches unitree_rl_gym normalization)
OBS_SCALE_ANG_VEL    = 0.25
OBS_SCALE_JOINT_POS  = 1.0
OBS_SCALE_JOINT_VEL  = 0.05
OBS_SCALE_ACTION     = 1.0
ACTION_SCALE         = 0.25  # output scale factor

# Gait parameters
GAIT_FREQ_HZ = 1.5   # step frequency


class G1LocomotionPolicy:
    """
    Wraps a pre-trained G1 RL locomotion policy.
    Falls back to a PD-only standing controller if no checkpoint is provided.
    """

    def __init__(self, checkpoint_path: str | None = None, device: str = "cpu"):
        self._device = device
        self._policy_net = None
        self._action_dim = NUM_POLICY_JOINTS
        self._obs_dim = 3 + 3 + 3 + NUM_POLICY_JOINTS * 3 + 4  # = 51

        self._prev_actions = np.zeros(NUM_POLICY_JOINTS, dtype=np.float32)
        self._gait_phase = 0.0
        self._policy_dt = 0.02  # 50Hz policy

        if checkpoint_path is not None:
            self._load_checkpoint(checkpoint_path)
        else:
            print("[Policy] No checkpoint provided — using PD standing controller.")

    def _load_checkpoint(self, path: str):
        if not _TORCH_AVAILABLE:
            print("[Policy] PyTorch not available — falling back to PD controller.")
            return

        try:
            ckpt = torch.load(path, map_location=self._device)

            # Handle different checkpoint formats
            if isinstance(ckpt, dict):
                if "actor" in ckpt:
                    state_dict = ckpt["actor"]
                elif "model_state_dict" in ckpt:
                    state_dict = ckpt["model_state_dict"]
                else:
                    state_dict = ckpt

                # Infer network dims from state dict
                first_key = next(iter(state_dict))
                self._obs_dim = state_dict[first_key].shape[1]
                last_key = [k for k in state_dict if "weight" in k][-1]
                self._action_dim = state_dict[last_key].shape[0]

                self._policy_net = self._build_mlp(self._obs_dim, self._action_dim)
                self._policy_net.load_state_dict(state_dict)
            else:
                # Direct module
                self._policy_net = ckpt

            self._policy_net.to(self._device)
            self._policy_net.eval()
            print(f"[Policy] Loaded checkpoint: obs={self._obs_dim} act={self._action_dim}")

        except Exception as e:
            print(f"[Policy] Failed to load checkpoint: {e}")
            print("[Policy] Falling back to PD standing controller.")
            self._policy_net = None

    @staticmethod
    def _build_mlp(obs_dim: int, act_dim: int) -> "nn.Module":
        return nn.Sequential(
            nn.Linear(obs_dim, 512), nn.ELU(),
            nn.Linear(512, 256),     nn.ELU(),
            nn.Linear(256, 128),     nn.ELU(),
            nn.Linear(128, act_dim),
        )

    def build_observation(
        self,
        ang_vel_base: np.ndarray,     # (3,) rad/s in base frame
        gravity_vec_base: np.ndarray, # (3,) gravity direction in base frame
        command: np.ndarray,          # (3,) [vx, vy, vyaw]
        joint_pos: np.ndarray,        # (14,) current joint positions
        joint_vel: np.ndarray,        # (14,) current joint velocities
    ) -> np.ndarray:
        """Build the 51-dim observation vector."""
        self._gait_phase += 2 * math.pi * GAIT_FREQ_HZ * self._policy_dt
        phase_obs = np.array([
            math.sin(self._gait_phase),
            math.cos(self._gait_phase),
            math.sin(self._gait_phase + math.pi),  # opposite phase for other leg
            math.cos(self._gait_phase + math.pi),
        ], dtype=np.float32)

        obs = np.concatenate([
            ang_vel_base    * OBS_SCALE_ANG_VEL,
            gravity_vec_base,
            command,
            (joint_pos - DEFAULT_JOINT_POS) * OBS_SCALE_JOINT_POS,
            joint_vel       * OBS_SCALE_JOINT_VEL,
            self._prev_actions * OBS_SCALE_ACTION,
            phase_obs,
        ]).astype(np.float32)

        return obs

    def step(
        self,
        ang_vel_base: np.ndarray,
        gravity_vec_base: np.ndarray,
        command: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ) -> np.ndarray:
        """
        Run one policy step.
        Returns target joint positions (14,) for PD control.
        """
        obs = self.build_observation(
            ang_vel_base, gravity_vec_base, command, joint_pos, joint_vel
        )

        if self._policy_net is not None and _TORCH_AVAILABLE:
            with torch.no_grad():
                obs_t = torch.tensor(obs, dtype=torch.float32, device=self._device).unsqueeze(0)
                action = self._policy_net(obs_t).squeeze(0).cpu().numpy()
        else:
            # PD standing: zero actions → hold default pose
            action = np.zeros(NUM_POLICY_JOINTS, dtype=np.float32)

        self._prev_actions = action.copy()

        # Scale and add to default joint positions
        target_joint_pos = DEFAULT_JOINT_POS + action * ACTION_SCALE
        return target_joint_pos

    def compute_torques(
        self,
        target_pos: np.ndarray,
        current_pos: np.ndarray,
        current_vel: np.ndarray,
    ) -> np.ndarray:
        """PD control: τ = Kp*(q_target - q) - Kd*dq"""
        return KP * (target_pos - current_pos) - KD * current_vel

    def reset(self):
        self._prev_actions = np.zeros(NUM_POLICY_JOINTS, dtype=np.float32)
        self._gait_phase = 0.0
