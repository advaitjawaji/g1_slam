import cv2
import numpy as np


class PreProcessor:
    def __init__(
        self,
        depth_threshold_m=3.0,
        alpha=0.35,
        invalid_frames_to_forget=5,
        warmup_frames=20,
        kernel_size=3,
    ):
        self.depth_threshold_m = depth_threshold_m
        self.alpha = alpha
        self.invalid_frames_to_forget = invalid_frames_to_forget
        self.warmup_frames = warmup_frames

        self.frame_idx = 0

        self.smoothed_depth = None
        self.smoothed_valid = None
        self.invalid_count = None

        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)

    def reset(self):
        self.frame_idx = 0
        self.smoothed_depth = None
        self.smoothed_valid = None
        self.invalid_count = None

    def _extract_depth(self, depth_frame: np.ndarray) -> np.ndarray:
        if depth_frame is None:
            raise ValueError("depth_frame is None")

        if depth_frame.ndim != 3 or depth_frame.shape[2] < 3:
            raise ValueError(
                f"depth_frame must have shape [H, W, >=3], got {depth_frame.shape}"
            )

        return depth_frame[:, :, 2].astype(np.float32)

    def _get_raw_valid_mask(self, depth_raw: np.ndarray):
        valid_raw = (
            np.isfinite(depth_raw)
            & (depth_raw > 0.0)
            & (depth_raw <= self.depth_threshold_m)
        )
        valid_ratio = float(np.count_nonzero(valid_raw)) / float(valid_raw.size)
        return valid_raw, valid_ratio

    def _initialize_buffers(self, depth_raw, valid_raw):
        self.smoothed_depth = np.zeros_like(depth_raw, dtype=np.float32)
        self.smoothed_valid = np.zeros_like(valid_raw, dtype=bool)
        self.invalid_count = np.zeros_like(depth_raw, dtype=np.uint8)

        self.smoothed_depth[valid_raw] = depth_raw[valid_raw]
        self.smoothed_valid[valid_raw] = True

    def _update_temporal_smoothing(self, depth_raw, valid_raw):
        if self.smoothed_depth is None:
            self._initialize_buffers(depth_raw, valid_raw)
            return

        both_valid = valid_raw & self.smoothed_valid
        self.smoothed_depth[both_valid] = (
            self.alpha * depth_raw[both_valid]
            + (1.0 - self.alpha) * self.smoothed_depth[both_valid]
        )

        new_valid = valid_raw & (~self.smoothed_valid)
        self.smoothed_depth[new_valid] = depth_raw[new_valid]
        self.smoothed_valid[new_valid] = True

        self.invalid_count[valid_raw] = 0

        invalid_now = ~valid_raw
        self.invalid_count[invalid_now] = np.minimum(
            self.invalid_count[invalid_now] + 1,
            self.invalid_frames_to_forget + 1,
        )

        forget_mask = self.invalid_count >= self.invalid_frames_to_forget
        self.smoothed_valid[forget_mask] = False
        self.smoothed_depth[forget_mask] = 0.0

    def _build_candidate_mask(self):
        valid_mask_candidate = (
            self.smoothed_valid
            & np.isfinite(self.smoothed_depth)
            & (self.smoothed_depth > 0.0)
            & (self.smoothed_depth <= self.depth_threshold_m)
        )

        mask_u8 = (valid_mask_candidate.astype(np.uint8)) * 255
        mask_u8 = cv2.medianBlur(mask_u8, 3)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        return mask_u8 > 0

    def process(self, image_bgr: np.ndarray, depth_frame: np.ndarray):
        if image_bgr is None:
            raise ValueError("image_bgr is None")

        self.frame_idx += 1

        depth_raw = self._extract_depth(depth_frame)
        valid_raw, valid_ratio = self._get_raw_valid_mask(depth_raw)

        # Warmup stage: show full image
        if self.frame_idx <= self.warmup_frames:
            full_mask = np.ones(image_bgr.shape[:2], dtype=bool)
            return {
                "masked_image": image_bgr.copy(),
                "mask": full_mask,
                "mask_u8": np.full(image_bgr.shape[:2], 255, dtype=np.uint8),
                "valid_ratio": valid_ratio,
                "use_depth_mask": False,
                "mask_status": "WARMUP",
                "is_warmup": True,
                "frame_idx": self.frame_idx,
            }

        self._update_temporal_smoothing(depth_raw, valid_raw)
        final_mask = self._build_candidate_mask()

        masked_image = image_bgr.copy()
        masked_image[~final_mask] = 0

        shown_mask = (final_mask.astype(np.uint8)) * 255

        return {
            "masked_image": masked_image,
            "mask": final_mask,
            "mask_u8": shown_mask,
            "valid_ratio": valid_ratio,
            "use_depth_mask": True,
            "mask_status": "DEPTH MASK ACTIVE",
            "is_warmup": False,
            "frame_idx": self.frame_idx,
        }