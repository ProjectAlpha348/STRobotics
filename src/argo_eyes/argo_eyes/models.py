from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any


@dataclass(frozen=True)
class EyeGeometry:
    center: Tuple[int, int]
    sclera_rx: int
    sclera_ry: int
    iris_rx: int
    iris_ry: int
    pupil_rx: int
    pupil_ry: int
    highlight_center: Tuple[int, int]
    highlight_rx: int
    highlight_ry: int


@dataclass
class EyeStateParams:
    open: float = 1.0
    saccades: bool = False
    pupil_scale: Tuple[float, float] = (1.0, 1.0)
    pupil_offset_left: Tuple[int, int] = (0, 0)
    pupil_offset_right: Tuple[int, int] = (0, 0)
    pupil_override: Optional[Tuple[int, int]] = None  # (rx, ry)
    pulse: bool = False


@dataclass
class BlinkConfig:
    enabled: bool
    interval_s: Tuple[float, float]
    frame_open: List[float]
    frame_dt_ms: int


@dataclass
class DisplayConfig:
    width: int
    height: int
    fps: int


@dataclass
class BehaviorConfig:
    default_state: str
    alert_pulse_dt_ms: int


@dataclass
class EyesConfig:
    display: DisplayConfig
    geometry: EyeGeometry
    states: Dict[str, EyeStateParams]
    blink: BlinkConfig
    behavior: BehaviorConfig
