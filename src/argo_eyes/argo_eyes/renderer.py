from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Optional

@dataclass(frozen=True)
class FramePlan:
    open: float
    pupil_rx: int
    pupil_ry: int
    pupil_center_left: Tuple[int, int]
    pupil_center_right: Tuple[int, int]
    pulse_on: bool = False

def compute_masks(eye_y0: int, eye_y1: int, eye_h: int, open_ratio: float) -> Tuple[Optional[Tuple[int,int]], Optional[Tuple[int,int]]]:
    """Return (top_mask_y0y1, bottom_mask_y0y1) as y-ranges to clear (black), or None if no mask."""
    if open_ratio >= 0.999:
        return None, None
    visible = int(round(eye_h * max(0.0, min(1.0, open_ratio))))
    cover = max(0, eye_h - visible)
    top = cover // 2
    bottom = cover - top
    top_mask = (0, eye_y0 + top)
    bottom_mask = (eye_y1 - bottom, 63)
    return top_mask, bottom_mask
