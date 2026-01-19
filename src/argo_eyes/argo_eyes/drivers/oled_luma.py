
from __future__ import annotations
from typing import Optional, Protocol

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306

class MuxLike(Protocol):
    def select(self, channel: int) -> None: ...

class OledDevice:
    def __init__(
        self,
        i2c_bus: int,
        address: int,
        driver: str = "sh1106",
        mux: Optional[MuxLike] = None,
        mux_channel: Optional[int] = None,
    ):
        self.address = int(address)
        self.driver = driver.lower().strip()
        self.mux = mux
        self.mux_channel = mux_channel

        # IMPORTANT: seleziona il canale PRIMA dell'init del display
        self._select_mux()

        serial = i2c(port=i2c_bus, address=self.address)
        if self.driver == "ssd1306":
            self.dev = ssd1306(serial)
        else:
            self.dev = sh1106(serial)

    def _select_mux(self):
        if self.mux is not None and self.mux_channel is not None:
            self.mux.select(int(self.mux_channel))

    def draw(self, draw_fn):
        # seleziona sempre il canale prima di ogni render
        self._select_mux()
        with canvas(self.dev) as draw:
            draw_fn(draw)
