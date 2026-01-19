from __future__ import annotations
from smbus2 import SMBus

class PCA9548A:
    """
    PCA9548A I2C multiplexer: selezione canale tramite single-byte write (1 << channel)
    """
    def __init__(self, bus: int, address: int = 0x70):
        self.bus_id = bus
        self.address = address
        self._bus = SMBus(bus)

    def select(self, channel: int) -> None:
        if channel < 0 or channel > 7:
            raise ValueError("PCA9548A channel must be 0..7")
        self._bus.write_byte(self.address, 1 << channel)
