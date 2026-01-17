#!/usr/bin/env python3
# package: argo_bot (o un pacchetto dedicato es. gpio_utils)
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import UInt32MultiArray
from interface.srv import GpioMode, GpioSet, GpioPulse

import lgpio

# Mappa convenzioni
MODE_INPUT  = 0
MODE_OUTPUT = 1

PULL_NONE = 0
PULL_UP   = 1
PULL_DOWN = 2

# Su Raspberry Pi 5 il controller RP1 è di solito gpiochip4
GPIOCHIP = 4

class GpioManager(Node):
    def __init__(self):
        super().__init__('gpio_manager')
        self.cb_group = ReentrantCallbackGroup()

        # Apri una volta sola il chip
        try:
            self.hchip = lgpio.gpiochip_open(GPIOCHIP)
        except Exception as e:
            raise RuntimeError(f"Impossibile aprire gpiochip{GPIOCHIP}: {e}")

        # Stato linee
        self.line_modes = {}            # line -> MODE_INPUT/MODE_OUTPUT
        self.line_claims = set()        # line prenotate
        self.debounce_ms = {}           # line -> debounce
        self._input_threads = {}        # line -> Thread
        self._input_thread_stop = {}    # line -> threading.Event

        # Publisher eventi input: [line, level, stamp_ns]
        self.pub_events = self.create_publisher(UInt32MultiArray, 'gpio/events', 10)

        # Servizi
        self.srv_mode  = self.create_service(GpioMode,  'gpio/mode',  self.on_mode,  callback_group=self.cb_group)
        self.srv_set   = self.create_service(GpioSet,   'gpio/set',   self.on_set,   callback_group=self.cb_group)
        self.srv_pulse = self.create_service(GpioPulse, 'gpio/pulse', self.on_pulse, callback_group=self.cb_group)

        self.get_logger().info(f'GPIO Manager attivo su gpiochip{GPIOCHIP}')

    # ---- Helpers ----------------------------------------------------------
    def _free_line(self, line: int):
        # stop eventuale thread input
        if line in self._input_thread_stop:
            self._input_thread_stop[line].set()
            self._input_thread_stop.pop(line, None)
        t = self._input_threads.pop(line, None)
        if t and t.is_alive():
            t.join(timeout=0.2)

        # libera la linea
        try:
            lgpio.gpio_free(self.hchip, line)
        except Exception:
            pass
        self.line_modes.pop(line, None)
        self.debounce_ms.pop(line, None)
        self.line_claims.discard(line)

    def _request_line_output(self, line: int):
        lgpio.gpio_claim_output(self.hchip, line)
        self.line_modes[line] = MODE_OUTPUT
        self.line_claims.add(line)

    def _request_line_input(self, line: int, pull: int, debounce_ms: int = 10, poll_interval_ms: int = 2):
        # Tenta di applicare il pull interno (dipende dalla versione/SoC)
        try:
            if pull == PULL_UP and hasattr(lgpio, "SET_PULL_UP"):
                lgpio.gpio_claim_input(self.hchip, line, lgpio.SET_PULL_UP)
            elif pull == PULL_DOWN and hasattr(lgpio, "SET_PULL_DOWN"):
                lgpio.gpio_claim_input(self.hchip, line, lgpio.SET_PULL_DOWN)
            else:
                lgpio.gpio_claim_input(self.hchip, line)
                if pull in (PULL_UP, PULL_DOWN):
                    self.get_logger().warn(
                        f'Pull interno non disponibile su GPIO {line}: uso floating. '
                        f'Considera una resistenza esterna o verifica versione lgpio/kernel.'
                    )
        except Exception as e:
            self.get_logger().warn(f'Impossibile impostare pull su GPIO {line}: {e}. Uso input floating.')
            lgpio.gpio_claim_input(self.hchip, line)

        self.line_modes[line] = MODE_INPUT
        self.line_claims.add(line)
        self.debounce_ms[line] = max(0, int(debounce_ms))

        # Avvia thread di polling con debounce software
        stop_evt = threading.Event()
        self._input_thread_stop[line] = stop_evt

        t = threading.Thread(
            target=self._input_poll_loop,
            args=(line, stop_evt, poll_interval_ms),
            daemon=True
        )
        self._input_threads[line] = t
        t.start()
        self.get_logger().info(f'Polling input avviato su GPIO {line} (debounce={debounce_ms} ms, poll={poll_interval_ms} ms)')

    def _input_poll_loop(self, line: int, stop_evt: threading.Event, poll_interval_ms: int):
        try:
            last_level = lgpio.gpio_read(self.hchip, line)
        except Exception:
            last_level = 0
        stable_level = last_level
        last_change_ns = time.time_ns()
        debounce_ns = self.debounce_ms.get(line, 10) * 1_000_000  # ms -> ns
        poll_s = max(0.0005, poll_interval_ms / 1000.0)

        while rclpy.ok() and not stop_evt.is_set():
            try:
                level = lgpio.gpio_read(self.hchip, line)
                now_ns = time.time_ns()

                if level != stable_level:
                    # livello diverso da quello stabile: avvia/aggiorna finestra debounce
                    if level != last_level:
                        last_change_ns = now_ns
                        last_level = level
                    else:
                        # stesso livello che persiste: se supera la finestra, conferma cambio
                        if now_ns - last_change_ns >= debounce_ns:
                            stable_level = level
                            # Pubblica evento
                            msg = UInt32MultiArray()
                            msg.data = [int(line), int(stable_level), int(now_ns)]
                            self.pub_events.publish(msg)
                else:
                    # livello uguale allo stabile: resetta eventuale rilevazione transitoria
                    last_level = level
                    last_change_ns = now_ns

            except Exception as e:
                self.get_logger().warn(f'Poll GPIO {line}: {e}')
                time.sleep(0.01)

            time.sleep(poll_s)

    # ---- Service callbacks ------------------------------------------------
    def on_mode(self, req: GpioMode.Request, resp: GpioMode.Response):
        line = int(req.line)
        try:
            # Se già richiesta, libera e ri-prenota
            if line in self.line_claims:
                self._free_line(line)

            if req.mode == MODE_OUTPUT:
                self._request_line_output(line)
            elif req.mode == MODE_INPUT:
                # debounce di default 10 ms; puoi renderlo parametro se vuoi
                self._request_line_input(line, req.pull, debounce_ms=10, poll_interval_ms=2)
            else:
                raise ValueError('mode non valido')

            resp.ok = True
            resp.msg = f'GPIO {line} configurata come {"OUTPUT" if req.mode==MODE_OUTPUT else "INPUT"}'
        except Exception as e:
            resp.ok = False
            resp.msg = f'Errore su GPIO {line}: {e}'
        return resp

    def on_set(self, req: GpioSet.Request, resp: GpioSet.Response):
        line = int(req.line)
        try:
            if self.line_modes.get(line) != MODE_OUTPUT:
                raise RuntimeError('Linea non in OUTPUT; chiama prima gpio/mode')

            lgpio.gpio_write(self.hchip, line, 1 if req.value else 0)
            resp.ok = True
            resp.msg = f'GPIO {line} -> {"HIGH" if req.value else "LOW"}'
        except Exception as e:
            resp.ok = False
            resp.msg = f'Errore set GPIO {line}: {e}'
        return resp

    def on_pulse(self, req: GpioPulse.Request, resp: GpioPulse.Response):
        line = int(req.line)
        try:
            if self.line_modes.get(line) != MODE_OUTPUT:
                raise RuntimeError('Linea non in OUTPUT; chiama prima gpio/mode')

            def _pulse():
                try:
                    for _ in range(int(req.repeat)):
                        lgpio.gpio_write(self.hchip, line, 1)
                        time.sleep(req.on_ms / 1000.0)
                        lgpio.gpio_write(self.hchip, line, 0)
                        time.sleep(req.off_ms / 1000.0)
                except Exception as e:
                    self.get_logger().error(f'Pulse errore su GPIO {line}: {e}')

            threading.Thread(target=_pulse, daemon=True).start()
            resp.ok = True
            resp.msg = f'Pulse avviato su GPIO {line}'
        except Exception as e:
            resp.ok = False
            resp.msg = f'Errore pulse GPIO {line}: {e}'
        return resp

    def destroy_node(self):
        # Cleanup linee e chip
        for line in list(self.line_claims):
            self._free_line(line)
        try:
            lgpio.gpiochip_close(self.hchip)
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = GpioManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()