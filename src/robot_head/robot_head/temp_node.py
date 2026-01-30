#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from interface.srv import Environment  # tuo pacchetto "interface"

# Adafruit BME280 (CircuitPython)
import board
import busio
import adafruit_bme280.advanced as adafruit_bme280


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')

        # ---------------- Parametri ----------------
        self.declare_parameter('i2c_address', 0x76)           # 0x76 o 0x77
        self.declare_parameter('update_hz', 2.0)              # aggiornamento cache + publish topic
        self.declare_parameter('sea_level_hpa', 1013.25)      # (opzionale)
        self.declare_parameter('service_name', '/tommy/environment')

        # NEW: topic env per orchestrator
        self.declare_parameter('env_topic', '/tommy/sensors/env')

        # NEW: log throttling errori lettura
        self.declare_parameter('error_throttle_s', 5.0)

        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.update_hz = float(self.get_parameter('update_hz').value)
        self.sea_level_hpa = float(self.get_parameter('sea_level_hpa').value)
        self.service_name = str(self.get_parameter('service_name').value)
        self.env_topic = str(self.get_parameter('env_topic').value)
        self.error_throttle_s = float(self.get_parameter('error_throttle_s').value)

        if self.update_hz <= 0.0:
            self.update_hz = 1.0

        if self.error_throttle_s <= 0.0:
            self.error_throttle_s = 5.0

        # ---------------- Init I2C + sensore ----------------
        self.bme280 = None
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=self.i2c_address)

            # Settaggi sensati
            self.bme280.mode = adafruit_bme280.MODE_NORMAL
            self.bme280.standby_period = adafruit_bme280.STANDBY_TC_250
            self.bme280.iir_filter = adafruit_bme280.IIR_FILTER_X4
            self.bme280.overscan_temperature = adafruit_bme280.OVERSCAN_X2
            self.bme280.overscan_pressure = adafruit_bme280.OVERSCAN_X4
            self.bme280.overscan_humidity = adafruit_bme280.OVERSCAN_X1

            self.get_logger().info(f'BME280 inizializzato su I2C addr=0x{self.i2c_address:02X}')
        except Exception as e:
            self.bme280 = None
            self.get_logger().error(f'Impossibile inizializzare BME280: {e}')

        # ---------------- Cache ultime letture ----------------
        self.last_temp_c: Optional[float] = None
        self.last_pres_hpa: Optional[float] = None
        self.last_hum_rh: Optional[float] = None
        self.last_stamp = self.get_clock().now()

        # throttling log errori
        self._last_err_log_ts = 0.0

        # ---------------- ROS: publisher + service ----------------
        self.env_pub = self.create_publisher(String, self.env_topic, 10)
        self.srv = self.create_service(Environment, self.service_name, self._on_service)

        # ---------------- Timer aggiornamento cache + publish ----------------
        period = 1.0 / self.update_hz
        self.timer = self.create_timer(period, self._update_cache_and_publish)

        self.get_logger().info(
            f'Service pronto: {self.service_name} | Topic env: {self.env_topic} '
            f'(update_hz={self.update_hz}, sea_level_hpa={self.sea_level_hpa})'
        )

    def _read_sensor(self) -> Tuple[float, float, float]:
        """
        Ritorna: (temp_C, pres_hPa, hum_RH)
        """
        if self.bme280 is None:
            raise RuntimeError("BME280 non inizializzato")

        temp_c = float(self.bme280.temperature)        # °C
        pres_hpa = float(self.bme280.pressure)         # hPa
        hum_rh = float(self.bme280.relative_humidity)  # %
        return temp_c, pres_hpa, hum_rh

    def _log_warn_throttled(self, text: str):
        now = time.time()
        if (now - self._last_err_log_ts) >= self.error_throttle_s:
            self._last_err_log_ts = now
            self.get_logger().warn(text)

    def _update_cache_and_publish(self):
        try:
            t, p, h = self._read_sensor()

            # sanity check (NaN/Inf)
            if any((math.isnan(x) or math.isinf(x)) for x in (t, p, h)):
                raise ValueError("lettura NaN/Inf")

            self.last_temp_c = t
            self.last_pres_hpa = p
            self.last_hum_rh = h
            self.last_stamp = self.get_clock().now()

            # Publish per orchestrator: "temp=23.4;hum=48.2"
            msg = String()
            msg.data = f"temp={t:.1f};hum={h:.1f}"
            self.env_pub.publish(msg)

        except Exception as e:
            self._log_warn_throttled(f'BME280 read failed: {e}')

    def _on_service(self, request: Environment.Request, response: Environment.Response):
        # Ignoro request.* (come facevi già)
        if self.last_temp_c is None or self.last_pres_hpa is None or self.last_hum_rh is None:
            response.ok = False
            response.data = 'Dati non disponibili (sensore non pronto o errore lettura).'
            return response

        age = (self.get_clock().now() - self.last_stamp).nanoseconds / 1e9

        response.ok = True
        response.data = (
            f"temp_C={self.last_temp_c:.2f}, "
            f"pres_hPa={self.last_pres_hpa:.2f}, "
            f"hum_RH={self.last_hum_rh:.2f}, "
            f"age_s={age:.2f}"
        )
        return response


def main():
    rclpy.init()
    node = EnvironmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
