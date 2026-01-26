#!/usr/bin/env python3
import time
import threading
import random
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smbus2 import SMBus

from luma.core.interface.serial import i2c
from luma.oled.device import sh1106, ssd1306
from luma.core.render import canvas


class TCA9548A:
    def __init__(self, bus: int, addr: int):
        self.bus_id = bus
        self.addr = addr
        self.bus = SMBus(bus)
        self._lock = threading.Lock()

    def select(self, channel: int):
        if channel < 0 or channel > 7:
            raise ValueError("TCA9548A channel must be in [0..7]")
        with self._lock:
            self.bus.write_byte(self.addr, 1 << channel)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


class EyeDevice:
    def __init__(self, tca: TCA9548A, channel: int, oled_addr: int, width: int, height: int, driver: str):
        self.tca = tca
        self.channel = channel
        self.oled_addr = oled_addr
        self.width = width
        self.height = height
        self.driver = driver
        self.device = None

    def open(self):
        self.tca.select(self.channel)
        serial = i2c(port=self.tca.bus_id, address=self.oled_addr)
        # driver: "sh1106" oppure "ssd1306"
        if self.driver.lower() == "ssd1306":
            self.device = ssd1306(serial, width=self.width, height=self.height)
        else:
            self.device = sh1106(serial, width=self.width, height=self.height)
        self.clear()

    def clear(self):
        if self.device is None:
            return
        self.tca.select(self.channel)
        self.device.clear()
        self.device.show()

    def draw(self, draw_fn):
        if self.device is None:
            return
        self.tca.select(self.channel)
        with canvas(self.device) as draw:
            draw_fn(draw)


class EyesNode(Node):
    def __init__(self):
        super().__init__("eyes_node")

        # Params
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("tca_addr", 0x70)
        self.declare_parameter("oled_addr", 0x3C)
        self.declare_parameter("left_channel", 0)
        self.declare_parameter("right_channel", 1)
        self.declare_parameter("width", 128)
        self.declare_parameter("height", 64)
        self.declare_parameter("driver", "sh1106")  # "sh1106" o "ssd1306"

        self.declare_parameter("blink_enabled", True)
        self.declare_parameter("blink_min_s", 3.0)
        self.declare_parameter("blink_max_s", 7.0)

        self.declare_parameter("pupil_offset_x", 0)  # per centrare pupilla
        self.declare_parameter("pupil_offset_y", 0)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.tca_addr = int(self.get_parameter("tca_addr").value)
        self.oled_addr = int(self.get_parameter("oled_addr").value)
        self.left_ch = int(self.get_parameter("left_channel").value)
        self.right_ch = int(self.get_parameter("right_channel").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.driver = str(self.get_parameter("driver").value)

        self.blink_enabled = bool(self.get_parameter("blink_enabled").value)
        self.blink_min_s = float(self.get_parameter("blink_min_s").value)
        self.blink_max_s = float(self.get_parameter("blink_max_s").value)

        self.pupil_off_x = int(self.get_parameter("pupil_offset_x").value)
        self.pupil_off_y = int(self.get_parameter("pupil_offset_y").value)

        # HW
        self.tca = TCA9548A(self.i2c_bus, self.tca_addr)
        self.left = EyeDevice(self.tca, self.left_ch, self.oled_addr, self.w, self.h, self.driver)
        self.right = EyeDevice(self.tca, self.right_ch, self.oled_addr, self.w, self.h, self.driver)

        self.left.open()
        self.right.open()

        # State
        self._state_lock = threading.Lock()
        self.state = "normal"
        self._running = True

        # Eye pose (pupils)
        self.pupil_x = 0
        self.pupil_y = 0

        # ROS
        self.sub = self.create_subscription(String, "/robot_head/eyes/cmd", self._on_cmd, 10)

        # Timers/threads
        self.render_timer = self.create_timer(0.05, self._render_tick)  # 20 FPS
        self._blink_thread = threading.Thread(target=self._blink_loop, daemon=True)
        self._blink_thread.start()

        self.get_logger().info("eyes_node avviato. Topic: /robot_head/eyes/cmd")

    # --------------------------
    # Commands
    # --------------------------
    def _on_cmd(self, msg: String):
        cmd = (msg.data or "").strip()
        if not cmd:
            return

        # Esempi:
        # "state=alert"
        # "blink=0"
        # "look=x:10,y:-5"
        # "center"
        if cmd == "center":
            self._set_look(0, 0)
            return

        m = re.match(r"^state\s*=\s*([a-zA-Z_]+)\s*$", cmd)
        if m:
            st = m.group(1).lower()
            if st == "stop":
                self.get_logger().info("Ricevuto state=stop: animazione spegnimento + shutdown.")
                self._tv_off_and_shutdown()
            else:
                self._set_state(st)
            return

        m = re.match(r"^blink\s*=\s*([01]|true|false)\s*$", cmd, re.IGNORECASE)
        if m:
            v = m.group(1).lower()
            self.blink_enabled = (v in ("1", "true"))
            self.get_logger().info(f"blink_enabled={self.blink_enabled}")
            return

        m = re.match(r"^look\s*=\s*x\s*:\s*(-?\d+)\s*,\s*y\s*:\s*(-?\d+)\s*$", cmd, re.IGNORECASE)
        if m:
            x = int(m.group(1))
            y = int(m.group(2))
            self._set_look(x, y)
            return

        self.get_logger().warn(f"Comando non riconosciuto: '{cmd}'")

    def _set_state(self, st: str):
        with self._state_lock:
            self.state = st

    def _set_look(self, x: int, y: int):
        # clamp leggero
        x = max(-20, min(20, x))
        y = max(-12, min(12, y))
        self.pupil_x = x
        self.pupil_y = y

    # --------------------------
    # Rendering
    # --------------------------
    def _render_tick(self):
        if not self._running:
            return

        with self._state_lock:
            st = self.state

        # Disegno base: sclera + pupilla
        def draw_eye(draw, mood: str, blink: float):
            # blink: 0=open, 1=closed
            cx = self.w // 2
            cy = self.h // 2

            # sclera
            rx, ry = 48, 22
            left = cx - rx
            top = cy - ry
            right = cx + rx
            bottom = cy + ry

            # eyelid (blink)
            if blink >= 1.0:
                # completamente chiuso: linea
                draw.line((left, cy, right, cy), fill=255, width=2)
                return
            elif blink > 0.0:
                # parzialmente chiuso: rettangoli sopra/sotto
                cover = int(ry * blink)
                draw.ellipse((left, top, right, bottom), outline=255, fill=0)
                draw.rectangle((left, top, right, top + cover), fill=0)
                draw.rectangle((left, bottom - cover, right, bottom), fill=0)
            else:
                draw.ellipse((left, top, right, bottom), outline=255, fill=0)

            # pupilla (con offset per centraggio)
            px = cx + self.pupil_x + self.pupil_off_x
            py = cy + self.pupil_y + self.pupil_off_y
            pr = 8

            # mood adjustments
            if mood == "alert":
                pr = 10
            elif mood == "attentive":
                pr = 7

            draw.ellipse((px - pr, py - pr, px + pr, py + pr), outline=255, fill=255)

            # sopracciglia / espressione semplice
            if mood == "alert":
                draw.line((cx - 35, cy - 25, cx - 5, cy - 30), fill=255, width=2)
                draw.line((cx + 5, cy - 30, cx + 35, cy - 25), fill=255, width=2)
            elif mood == "off":
                draw.line((cx - 25, cy, cx + 25, cy), fill=255, width=2)

        # Blink state (gestito dal thread)
        blink = getattr(self, "_blink_amount", 0.0)

        mood = st
        if st not in ("normal", "alert", "attentive", "off"):
            mood = "normal"

        # Render entrambi
        self.left.draw(lambda d: draw_eye(d, mood, blink))
        self.right.draw(lambda d: draw_eye(d, mood, blink))

    # --------------------------
    # Blink loop
    # --------------------------
    def _blink_loop(self):
        self._blink_amount = 0.0
        while self._running:
            if not self.blink_enabled:
                self._blink_amount = 0.0
                time.sleep(0.1)
                continue

            # attesa random
            wait_s = random.uniform(self.blink_min_s, self.blink_max_s)
            t0 = time.time()
            while self._running and (time.time() - t0) < wait_s:
                time.sleep(0.05)

            if not self._running:
                break

            # chiudi-apri
            for a in [0.4, 0.8, 1.0]:
                self._blink_amount = a
                time.sleep(0.03)
            for a in [0.8, 0.4, 0.0]:
                self._blink_amount = a
                time.sleep(0.04)

    # --------------------------
    # TV off animation + shutdown
    # --------------------------
    def _tv_off_and_shutdown(self):
        # animazione: da ellisse piena -> linea -> punto -> off
        def frame(hh, ww):
            cx = self.w // 2
            cy = self.h // 2
            x0 = cx - ww // 2
            x1 = cx + ww // 2
            y0 = cy - hh // 2
            y1 = cy + hh // 2
            def _draw(draw):
                draw.rectangle((0, 0, self.w, self.h), fill=0)
                if hh <= 2:
                    draw.line((x0, cy, x1, cy), fill=255, width=2)
                else:
                    draw.rectangle((x0, y0, x1, y1), outline=255, fill=0)
            return _draw

        for ww, hh in [(96, 28), (80, 18), (60, 10), (40, 6), (20, 3), (8, 2), (2, 2)]:
            self.left.draw(frame(hh, ww))
            self.right.draw(frame(hh, ww))
            time.sleep(0.05)

        # spegni
        self.left.clear()
        self.right.clear()

        # stop node
        self._running = False
        # shutdown ROS in modo pulito
        threading.Thread(target=self._shutdown_ros, daemon=True).start()

    def _shutdown_ros(self):
        time.sleep(0.1)
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def destroy_node(self):
        self._running = False
        try:
            self.left.clear()
            self.right.clear()
        except Exception:
            pass
        try:
            self.tca.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = EyesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info("Chiusura eyes_node...")
        node.destroy_node()


if __name__ == "__main__":
    main()
