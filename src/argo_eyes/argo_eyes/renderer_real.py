from __future__ import annotations
from typing import Tuple, Optional

from .models import EyeGeometry, EyeStateParams
from .renderer import FramePlan, compute_masks

def _ellipse_bbox(center: Tuple[int,int], rx: int, ry: int) -> Tuple[int,int,int,int]:
    cx, cy = center
    return (cx - rx, cy - ry, cx + rx, cy + ry)

def draw_eye_frame(
    draw,
    geom: EyeGeometry,
    plan: FramePlan,
    params: EyeStateParams,
    is_left: bool,
):
    """
    draw: PIL.ImageDraw (da luma.canvas)
    ON=verde => fill=255, OFF=nero => fill=0
    """
    # Costanti ON/OFF (monocromatico)
    ON = 255
    OFF = 0

    cx, cy = geom.center

    # Pupilla centers differenziati (left/right)
    pupil_center = plan.pupil_center_left if is_left else plan.pupil_center_right

    # 1) Sclera ON
    draw.ellipse(_ellipse_bbox((cx, cy), geom.sclera_rx, geom.sclera_ry), fill=ON)

    # 2) Iride ON (ellittica tech)
    draw.ellipse(_ellipse_bbox((cx, cy + 2), geom.iris_rx, geom.iris_ry), fill=ON)

    # 3) Pupilla OFF (nero)
    draw.ellipse(_ellipse_bbox(pupil_center, plan.pupil_rx, plan.pupil_ry), fill=OFF)

    # 4) Highlight ON
    draw.ellipse(_ellipse_bbox(geom.highlight_center, geom.highlight_rx, geom.highlight_ry), fill=ON)

    # 5) Palpebre (maschere nere)
    # bounding y occhio (sclera)
    eye_y0 = cy - geom.sclera_ry
    eye_y1 = cy + geom.sclera_ry
    eye_h = 2 * geom.sclera_ry

    top_mask, bottom_mask = compute_masks(eye_y0, eye_y1, eye_h, plan.open)
    if top_mask is not None:
        y0, y1 = top_mask
        draw.rectangle((0, y0, 127, y1), fill=OFF)
    if bottom_mask is not None:
        y0, y1 = bottom_mask
        draw.rectangle((0, y0, 127, y1), fill=OFF)

    # 6) Pulse in alert: semplice “flash” (riduce sclera visibile quando pulse_off)
    # (qui non cambia colore, cambia intensità percepita con un frame alternato)
    if plan.pulse_on is False and params.pulse:
        # oscura un velo leggero (banda superiore) per far percepire "pulsazione"
        draw.rectangle((0, 0, 127, 10), fill=OFF)
