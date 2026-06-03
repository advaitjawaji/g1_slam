import time
from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

try:
    import torch
except ImportError:  
    torch = None

from g1_detection.eigen import EigenTrajectoryAdapter


class TrajectoryFusion:


    def __init__(
        self,
        eigen_adapter: Optional[EigenTrajectoryAdapter] = None,
        eigen_weight: float = 0.65,
        max_eigen_jump_m: float = 2.5,
    ):
        self.eigen_adapter = eigen_adapter if eigen_adapter is not None else EigenTrajectoryAdapter()
        self.eigen_weight = float(np.clip(eigen_weight, 0.0, 1.0))
        self.detector_weight = 1.0 - self.eigen_weight
        self.max_eigen_jump_m = float(max_eigen_jump_m)

    def setup_eigen(
        self,
        hyper_params,
        checkpoint_path: Optional[str] = None,
        device: str = "cpu",
        obs_len: int = 8,
        pred_len: int = 12,
        infer_rate_hz: float = 10.0,
    ) -> None:
        self.eigen_adapter.setup(
            hyper_params=hyper_params,
            checkpoint_path=checkpoint_path,
            device=device,
            obs_len=obs_len,
            pred_len=pred_len,
            infer_rate_hz=infer_rate_hz,
        )

    def update_observation(self, track_id: int, x: float, z: float) -> None:
        self.eigen_adapter.update_history(track_id, np.array([x, z], dtype=np.float32))

    @staticmethod
    def _heuristic_to_array(predictions: List[Tuple[float, float, float]]) -> Tuple[np.ndarray, List[float]]:
        if not predictions:
            return np.empty((0, 2), dtype=np.float32), []
        arr = np.asarray([(p[0], p[1]) for p in predictions], dtype=np.float32)
        times = [float(p[2]) for p in predictions]
        return arr, times

    @staticmethod
    def _resample_xy(pred_xy: np.ndarray, target_len: int) -> np.ndarray:
        pred_xy = np.asarray(pred_xy, dtype=np.float32)
        if target_len <= 0 or pred_xy.size == 0:
            return np.empty((0, 2), dtype=np.float32)
        if pred_xy.ndim != 2 or pred_xy.shape[1] != 2:
            return np.empty((0, 2), dtype=np.float32)
        if len(pred_xy) == target_len:
            return pred_xy
        if len(pred_xy) == 1:
            return np.repeat(pred_xy, target_len, axis=0)

        src = np.linspace(0.0, 1.0, len(pred_xy))
        dst = np.linspace(0.0, 1.0, target_len)
        x = np.interp(dst, src, pred_xy[:, 0])
        z = np.interp(dst, src, pred_xy[:, 1])
        return np.stack([x, z], axis=1).astype(np.float32)

    def _valid_eigen(self, eigen_xy: Optional[np.ndarray], current_xz: Tuple[float, float]) -> bool:
        if eigen_xy is None:
            return False
        eigen_xy = np.asarray(eigen_xy)
        if eigen_xy.ndim != 2 or eigen_xy.shape[1] != 2 or len(eigen_xy) == 0:
            return False
        if not np.isfinite(eigen_xy).all():
            return False
        first_jump = float(np.linalg.norm(eigen_xy[0] - np.asarray(current_xz, dtype=np.float32)))
        return first_jump <= self.max_eigen_jump_m

    def fuse(
        self,
        track_id: int,
        current_x: float,
        current_z: float,
        heuristic_predictions: List[Tuple[float, float, float]],
        timestamp: float,
        prediction_steps: int,
        prediction_secs: float,
    ) -> Dict[str, object]:
        self.eigen_adapter.maybe_predict()
        eigen_xy = self.eigen_adapter.get_prediction_for_track(track_id)

        heuristic_xy, heuristic_times = self._heuristic_to_array(heuristic_predictions)
        eigen_ok = self._valid_eigen(eigen_xy, (current_x, current_z))

        if len(heuristic_xy) > 0:
            target_len = len(heuristic_xy)
            times = heuristic_times
        elif eigen_ok:
            target_len = min(int(prediction_steps), len(eigen_xy))
            step_dt = float(prediction_secs) / max(target_len, 1)
            times = [float(timestamp + (i + 1) * step_dt) for i in range(target_len)]
        else:
            target_len = 0
            times = []

        if target_len == 0:
            fused_xy = np.empty((0, 2), dtype=np.float32)
            method = "none"
        elif eigen_ok and len(heuristic_xy) > 0:
            eigen_rs = self._resample_xy(eigen_xy, target_len)
            fused_xy = self.detector_weight * heuristic_xy + self.eigen_weight * eigen_rs
            method = "weighted_detector_eigen"
        elif eigen_ok:
            fused_xy = self._resample_xy(eigen_xy, target_len)
            method = "eigen_only"
        else:
            fused_xy = heuristic_xy
            method = "detector_only"

        fused = [(float(x), float(z), float(t)) for (x, z), t in zip(fused_xy, times)]
        eigen_list = []
        if eigen_ok:
            eigen_rs_for_output = self._resample_xy(eigen_xy, target_len if target_len else len(eigen_xy))
            eigen_times = times if target_len else [float(timestamp)] * len(eigen_rs_for_output)
            eigen_list = [
                (float(x), float(z), float(t))
                for (x, z), t in zip(eigen_rs_for_output, eigen_times)
            ]

        return {
            "method": method,
            "final": fused,
            "detector": heuristic_predictions,
            "eigen": eigen_list,
            "eigen_available": bool(eigen_ok),
        }

    def reset_track(self, track_id: int) -> None:
        self.eigen_adapter.reset_track(track_id)

    def reset(self) -> None:
        self.eigen_adapter.reset()


