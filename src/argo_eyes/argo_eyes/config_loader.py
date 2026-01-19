from __future__ import annotations
from typing import Any, Dict, Tuple, List, Optional
import yaml

from .models import (
    EyesConfig, DisplayConfig, EyeGeometry, EyeStateParams,
    BlinkConfig, BehaviorConfig
)

def _t2(v) -> Tuple[int, int]:
    return (int(v[0]), int(v[1]))

def load_eyes_config(path: str) -> EyesConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    disp = raw["display"]
    display = DisplayConfig(
        width=int(disp["width"]),
        height=int(disp["height"]),
        fps=int(disp.get("fps", 30)),
    )

    geo = raw["geometry"]
    geometry = EyeGeometry(
        center=_t2(geo["center"]),
        sclera_rx=int(geo["sclera_rx"]),
        sclera_ry=int(geo["sclera_ry"]),
        iris_rx=int(geo["iris_rx"]),
        iris_ry=int(geo["iris_ry"]),
        pupil_rx=int(geo["pupil_rx"]),
        pupil_ry=int(geo["pupil_ry"]),
        highlight_center=_t2(geo["highlight_center"]),
        highlight_rx=int(geo["highlight_rx"]),
        highlight_ry=int(geo["highlight_ry"]),
    )

    states: Dict[str, EyeStateParams] = {}
    for name, s in raw.get("states", {}).items():
        p = EyeStateParams()
        if "open" in s: p.open = float(s["open"])
        if "saccades" in s: p.saccades = bool(s["saccades"])
        if "pupil_scale" in s:
            p.pupil_scale = (float(s["pupil_scale"][0]), float(s["pupil_scale"][1]))
        if "pupil_offset_left" in s:
            p.pupil_offset_left = (int(s["pupil_offset_left"][0]), int(s["pupil_offset_left"][1]))
        if "pupil_offset_right" in s:
            p.pupil_offset_right = (int(s["pupil_offset_right"][0]), int(s["pupil_offset_right"][1]))
        if "pupil_override" in s:
            po = s["pupil_override"]
            p.pupil_override = (int(po["rx"]), int(po["ry"]))
        if "pulse" in s:
            p.pulse = bool(s["pulse"])
        states[name] = p

    blink_raw = raw.get("blink", {})
    blink = BlinkConfig(
        enabled=bool(blink_raw.get("enabled", True)),
        interval_s=(float(blink_raw.get("interval_s", [3.0, 6.0])[0]),
                    float(blink_raw.get("interval_s", [3.0, 6.0])[1])),
        frame_open=[float(x) for x in blink_raw.get("frame_open", [1.0])],
        frame_dt_ms=int(blink_raw.get("frame_dt_ms", 30)),
    )

    beh = raw.get("behavior", {})
    behavior = BehaviorConfig(
        default_state=str(beh.get("default_state", "neutral")),
        alert_pulse_dt_ms=int(beh.get("alert_pulse_dt_ms", 180)),
    )

    return EyesConfig(
        display=display,
        geometry=geometry,
        states=states,
        blink=blink,
        behavior=behavior,
    )
