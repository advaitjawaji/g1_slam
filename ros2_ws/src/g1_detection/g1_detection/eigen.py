from __future__ import annotations

from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple
import time
import numpy as np

try:
    import torch
except ImportError:
    torch = None

try:
    from g1_detection.EigenTrajectory.eigen import EigenTrajectoryInference
except (ImportError, SystemExit):
    EigenTrajectoryInference = None


class EigenTrajectoryAdapter:
    def __init__(self):
        self._enabled = False
        self._device = "cpu"
        self._predictor = None

        self._obs_len = 8
        self._pred_len = 12
        self._num_samples = 1

        self._track_history: Dict[int, deque] = defaultdict(lambda: deque(maxlen=self._obs_len))
        self._last_predictions: Dict[int, np.ndarray] = {}
        self._last_infer_time_ns: int = 0
        self._infer_period_ns: int = int(1e9 / 10.0)  # 10 Hz

    def setup(
        self,
        hyper_params,
        checkpoint_path: Optional[str] = None,
        device: str = "cpu",
        obs_len: int = 8,
        pred_len: int = 12,
        infer_rate_hz: float = 10.0,
    ) -> None:
        if torch is None:
            raise ImportError("PyTorch is required for EigenTrajectoryAdapter.")

        if EigenTrajectoryInference is None:
            raise ImportError(
                "import error"
                )

        self._obs_len = obs_len
        self._pred_len = pred_len
        self._infer_period_ns = int(1e9 / infer_rate_hz)
        self._device = device

        self._predictor = EigenTrajectoryInference(hyper_params).to(device)

        if checkpoint_path is not None:
            checkpoint = torch.load(checkpoint_path, map_location=device)
            state_dict = checkpoint.get("model_state_dict", checkpoint)
            self._predictor.load_state_dict(state_dict)

        self._predictor.eval()
        self._enabled = True

    def update_history(
        self,
        track_id: int,
        position_xz: np.ndarray,
    ) -> None:
        self._track_history[track_id].append(position_xz.astype(np.float32))

    def maybe_predict(
        self,
        timestamp_ns: Optional[int] = None,
    ) -> Dict[int, np.ndarray]:
        if not self._enabled or self._predictor is None:
            return {}

        if timestamp_ns is None:
            timestamp_ns = time.time_ns()

        if (timestamp_ns - self._last_infer_time_ns) < self._infer_period_ns:
            return self._last_predictions

        eligible_ids: List[int] = []
        batch_obs: List[np.ndarray] = []

        for track_id, hist in self._track_history.items():
            if len(hist) == self._obs_len:
                eligible_ids.append(track_id)
                batch_obs.append(np.stack(hist, axis=0))  # [8,2]

        if len(batch_obs) == 0:
            return self._last_predictions

        obs_arr = np.stack(batch_obs, axis=0)  

        with torch.no_grad():
            obs_tensor = torch.tensor(obs_arr, dtype=torch.float32, device=self._device)
            preds = self._predictor(obs_tensor)  
            best_pred = preds[0].detach().cpu().numpy()  

        current_preds: Dict[int, np.ndarray] = {}
        for i, track_id in enumerate(eligible_ids):
            current_preds[track_id] = best_pred[i]

        self._last_predictions = current_preds
        self._last_infer_time_ns = timestamp_ns
        return self._last_predictions

    def get_prediction_for_track(self, track_id: int) -> Optional[np.ndarray]:
        return self._last_predictions.get(track_id, None)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def remove_track(self, track_id: int) -> None:
        self._track_history.pop(track_id, None)
        self._last_predictions.pop(track_id, None)

    def reset_track(self, track_id: int) -> None:
        self.remove_track(track_id)

    def reset(self) -> None:
        self._track_history.clear()
        self._last_predictions.clear()
        self._last_infer_time_ns = 0