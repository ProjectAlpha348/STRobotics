from __future__ import annotations

import os
import random
import time
from typing import Dict, Optional, Tuple
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .config_loader import load_eyes_config
from .models import EyesConfig, EyeStateParams
from .renderer import FramePlan

from typing import Any
from .drivers.pca9548a import PCA9548A
from .drivers.oled_luma import OledDevice
from .renderer_real import draw_eye_frame


PRIORITY = {
    "disabled": 1000,
    "alert": 400,
    "attentive": 300,
    "neutral": 200,
    "sleep": 100,
}

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

class EyesNode(Node):
    def __init__(self):
        super().__init__("argo_eyes")

        self.declare_parameter("config", "")
        cfg_path = self.get_parameter("config").get_parameter_value().string_value

        if not cfg_path:
            share = get_package_share_directory("argo_eyes")
            cfg_path = os.path.join(share, "config", "eyes.yaml")


        self.cfg: EyesConfig = load_eyes_config(cfg_path)
        # ===== ALERT AUTO-RESET (punto 2A) =====
        self.state_before_alert = self.cfg.behavior.default_state
        self.alert_until_t = 0.0

        # durata alert (secondi)
        self.alert_timeout_s = getattr(
            self.cfg.behavior, "alert_timeout_s", 3.0
        )

        # stato di fallback se qualcosa va storto
        self.alert_fallback_state = getattr(
            self.cfg.behavior, "alert_fallback_state", "neutral"
        )
        # --- OLED init (punto 3) ---
        disp_raw = self._load_display_raw(cfg_path)  # helper sotto
        self.oled_left = None
        self.oled_right = None

        mux = None
        mux_cfg = disp_raw.get("display", {}).get("mux")
        if mux_cfg and mux_cfg.get("type", "").lower() == "pca9548a":
           mux = PCA9548A(
               bus=int(disp_raw["display"].get("i2c_bus", 1)),
               address=int(mux_cfg.get("address", 0x70)),
           )


        i2c_bus = int(disp_raw["display"].get("i2c_bus", 1))
        driver = str(disp_raw["display"].get("driver", "sh1106"))

        devs = disp_raw["display"]["devices"]
        # ci aspettiamo left/right
        dleft = next(d for d in devs if d["name"] == "left")
        dright = next(d for d in devs if d["name"] == "right")

        self.oled_left = OledDevice(
            i2c_bus=i2c_bus,
            address=int(dleft["address"], 16) if isinstance(dleft["address"], str) else int(dleft["address"]),
            driver=driver,
            mux=mux,
            mux_channel=dleft.get("mux_channel"),
        )
        self.oled_right = OledDevice(
            i2c_bus=i2c_bus,
            address=int(dright["address"], 16) if isinstance(dright["address"], str) else int(dright["address"]),
            driver=driver,
            mux=mux,
            mux_channel=dright.get("mux_channel"),
        )


        self.cmd_sub = self.create_subscription(String, "/argo/eyes/cmd", self.on_cmd, 10)
        self.state_pub = self.create_publisher(String, "/argo/eyes/state", 10)

        self.current_state: str = self.cfg.behavior.default_state
        self.target_state: str = self.current_state
        self.enabled: bool = True

        # Blink overlay
        self.blink_active: bool = False
        self.blink_idx: int = 0
        self.blink_next_t: float = time.time() + self._rand_blink_interval()
        self.blink_frame_t: float = 0.0

        # Alert pulse
        self.pulse_on: bool = False
        self.pulse_next_t: float = time.time() + (self.cfg.behavior.alert_pulse_dt_ms / 1000.0)

        # main loop
        dt = 1.0 / float(self.cfg.display.fps)
        self.timer = self.create_timer(dt, self.tick)

        self.get_logger().info(f"argo_eyes ready. config={cfg_path} fps={self.cfg.display.fps}")

    def _rand_blink_interval(self) -> float:
        a, b = self.cfg.blink.interval_s
        return random.uniform(a, b)

    def _get_params(self, state: str) -> EyeStateParams:
        return self.cfg.states.get(state, EyeStateParams())

    def _set_state(self, new_state: str):
        new_state = new_state.strip().lower()
        if new_state not in PRIORITY:
            # se non è nel dict PRIORITY, lo accettiamo comunque ma con priorità neutra
            PRIORITY.setdefault(new_state, 200)

        # regola priorità: se il current è più alto, non scendere se non comandato esplicitamente
        self.target_state = new_state

        # transizione immediata se target ha priorità >= current o se stiamo uscendo da alert con comando esplicito
        if PRIORITY[new_state] >= PRIORITY.get(self.current_state, 0) or new_state != self.current_state:
            self.current_state = new_state

        # se entri in alert: forza blink off e avvia pulse
        if new_state == "alert":
            # salva stato precedente solo se stai entrando in alert da uno stato diverso
            if self.current_state != "alert":
                self.state_before_alert = self.current_state

            self.current_state = "alert"
            self.blink_active = False
            self.pulse_on = True
            self.pulse_next_t = time.time() + (self.cfg.behavior.alert_pulse_dt_ms / 1000.0)

            # timeout alert (refreshabile)
            self.alert_until_t = time.time() + float(getattr(self.cfg.behavior, "alert_timeout_s", 3.0))
            return

    def on_cmd(self, msg: String):
        s = (msg.data or "").strip()
        if not s:
            return

        # formato: "k=v k=v"
        parts = s.replace(",", " ").split()
        kv = {}
        for p in parts:
            if "=" in p:
                k, v = p.split("=", 1)
                kv[k.strip().lower()] = v.strip()
            else:
                # comando singolo (es: "blink")
                kv[p.strip().lower()] = "1"

        if "enable" in kv:
            self.enabled = kv["enable"] not in ("0", "false", "off", "no")
        if "state" in kv:
            self._set_state(kv["state"])
        if "blink" in kv:
            if kv["blink"] in ("once", "1", "true", "on"):
                self.start_blink()
        if "reset" in kv:
            self._set_state(self.cfg.behavior.default_state)
            return
        # look=x,y (opzionale futuro)
        # if "look" in kv: ...

    def start_blink(self):
        if not self.cfg.blink.enabled:
            return
        # non blinkare in alert (di default)
        if self.current_state == "alert":
            return
        self.blink_active = True
        self.blink_idx = 0
        self.blink_frame_t = time.time()

    def tick(self):
        now = time.time()
        # auto reset alert
        if self.current_state == "alert" and self.alert_until_t > 0.0 and now >= self.alert_until_t:
            fallback = getattr(self.cfg.behavior, "alert_fallback_state", "neutral")
            back = self.state_before_alert if self.state_before_alert else fallback
            if back == "alert":
                back = fallback
            self.current_state = back
            self.alert_until_t = 0.0

        if not self.enabled or self.current_state == "disabled":
            self._publish_state("disabled", open_ratio=0.0)
            return

        # pulse in alert
        if self.current_state == "alert" and now >= self.pulse_next_t:
            self.pulse_on = not self.pulse_on
            self.pulse_next_t = now + (self.cfg.behavior.alert_pulse_dt_ms / 1000.0)

        # auto blink scheduling (solo se abilitato dallo stato)
        params = self._get_params(self.current_state)
        if self.cfg.blink.enabled and params.saccades:
            if (not self.blink_active) and (now >= self.blink_next_t):
                self.start_blink()
                self.blink_next_t = now + self._rand_blink_interval()


        open_ratio = self._compute_open_ratio(now)
        plan = self._compute_frame_plan(open_ratio=open_ratio)

        # TODO (punto 3): inviare plan al driver OLED
        params = self._get_params(self.current_state)

        # Render su OLED sinistro
        self.oled_left.draw(lambda d: draw_eye_frame(d, self.cfg.geometry, plan, params, is_left=True))

        # Render su OLED destro
        self.oled_right.draw(lambda d: draw_eye_frame(d, self.cfg.geometry, plan, params, is_left=False))

        # per ora pubblichiamo solo stato
        extra = f" pulse={int(plan.pulse_on)} blink={int(self.blink_active)}"
        self._publish_state(self.current_state, open_ratio=open_ratio, extra=extra)

        # avanzamento blink
        if self.blink_active:
            dt_s = self.cfg.blink.frame_dt_ms / 1000.0
            if (now - self.blink_frame_t) >= dt_s:
                self.blink_idx += 1
                self.blink_frame_t = now
                if self.blink_idx >= len(self.cfg.blink.frame_open):
                    self.blink_active = False
                    self.blink_idx = 0

    def _compute_open_ratio(self, now: float) -> float:
        params = self._get_params(self.current_state)
        base_open = _clamp(params.open, 0.0, 1.0)

        if self.blink_active:
            seq = self.cfg.blink.frame_open
            idx = max(0, min(len(seq) - 1, self.blink_idx))
            return _clamp(seq[idx], 0.0, 1.0)

        # alert: potresti voler “stringere” quando pulse_off
        if self.current_state == "alert" and params.pulse:
            # quando pulse_on=0 potresti chiudere leggermente o cambiare look
            return base_open

        return base_open

    def _compute_frame_plan(self, open_ratio: float) -> FramePlan:
        g = self.cfg.geometry
        p = self._get_params(self.current_state)

        # Pupilla base
        rx = g.pupil_rx
        ry = g.pupil_ry

        # Override (alert)
        if p.pupil_override is not None:
            rx, ry = p.pupil_override

        # Scale (attentive)
        rx = int(round(rx * p.pupil_scale[0]))
        ry = int(round(ry * p.pupil_scale[1]))

        cx, cy = g.center[0], g.center[1] + 2  # pupilla leggermente più bassa (come definito)
        lx, ly = p.pupil_offset_left
        rx_off, ry_off = p.pupil_offset_right

        pupil_left = (cx + lx, cy + ly)
        pupil_right = (cx + rx_off, cy + ry_off)

        return FramePlan(
            open=open_ratio,
            pupil_rx=rx,
            pupil_ry=ry,
            pupil_center_left=pupil_left,
            pupil_center_right=pupil_right,
            pulse_on=(self.pulse_on if self.current_state == "alert" else False),
        )

    def _publish_state(self, state: str, open_ratio: float, extra: str = ""):
        msg = String()
        msg.data = f"state={state} open={open_ratio:.2f}{extra}"
        self.state_pub.publish(msg)

    def _load_display_raw(self, cfg_path: str) -> dict:
        import yaml
        with open(cfg_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}


def main():
    rclpy.init()
    node = EyesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
