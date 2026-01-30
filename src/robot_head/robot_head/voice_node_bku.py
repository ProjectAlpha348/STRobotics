#!/usr/bin/env python3
import json
import queue
import threading
import time
import audioop
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
from vosk import Model, KaldiRecognizer

from ament_index_python.packages import get_package_share_directory


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # ---------------- Parameters ----------------
        # Se vuoto, viene calcolato da share/robot_head/models/vosk/...
        self.declare_parameter('vosk_model_rel', 'models/vosk/vosk-model-small-it-0.22')
        self.declare_parameter('model_path', '')  # override opzionale

        # Il tuo device in hw supporta RATE: 44100/48000, S16_LE, mono
        self.declare_parameter('device', -1)                 # sounddevice index, -1=default
        self.declare_parameter('input_sample_rate', 48000)   # 44100 o 48000
        self.declare_parameter('vosk_sample_rate', 16000)    # consigliato per Vosk
        self.declare_parameter('channels', 1)
        self.declare_parameter('enable_partial', False)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('min_text_len', 2)
        self.declare_parameter('resample_to_vosk_rate', True)

        pkg_share = get_package_share_directory('robot_head')
        vosk_model_rel = self.get_parameter('vosk_model_rel').value
        model_path_override = (self.get_parameter('model_path').value or "").strip()

        self.model_path = model_path_override if model_path_override else f"{pkg_share}/{vosk_model_rel}"

        self.device = int(self.get_parameter('device').value)
        self.input_sr = int(self.get_parameter('input_sample_rate').value)
        self.vosk_sr = int(self.get_parameter('vosk_sample_rate').value)
        self.channels = int(self.get_parameter('channels').value)
        self.enable_partial = bool(self.get_parameter('enable_partial').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.min_text_len = int(self.get_parameter('min_text_len').value)
        self.do_resample = bool(self.get_parameter('resample_to_vosk_rate').value)

        if self.channels != 1:
            raise RuntimeError("Questo nodo è configurato per audio mono (channels=1).")

        if self.input_sr not in (44100, 48000):
            self.get_logger().warn(
                f"input_sample_rate={self.input_sr} non tipico per hw:1,0. "
                "Con il tuo microfono sono attesi 44100 o 48000."
            )

        # ---------------- Vosk ----------------
        self.get_logger().info(f"Carico modello Vosk da: {self.model_path}")
        self.model = Model(self.model_path)

        recognizer_rate = self.vosk_sr if (self.do_resample and self.input_sr != self.vosk_sr) else self.input_sr
        self.rec = KaldiRecognizer(self.model, float(recognizer_rate))
        self.rec.SetWords(True)

        self.get_logger().info(
            f"Audio IN: {self.input_sr} Hz, mono, S16_LE | "
            f"Vosk: {recognizer_rate} Hz | resample={self.do_resample}"
        )

        # ---------------- ROS pubs/subs ----------------
        self.pub_text = self.create_publisher(String, '/tommy/voice/text', 10)
        self.pub_partial = self.create_publisher(String, '/tommy/voice/partial', 10) if self.enable_partial else None
        self.pub_status = self.create_publisher(String, '/tommy/voice/status', 10)
        self.sub_cmd = self.create_subscription(String, '/tommy/voice/cmd', self._on_cmd, 10)
        self.sys_sub = self.create_subscription(String, "/robot_head/system/cmd", self._on_system_cmd, 10)

        # ---------------- Runtime state ----------------
        self._audio_q: "queue.Queue[bytes]" = queue.Queue(maxsize=100)
        self._stream: Optional[sd.RawInputStream] = None
        self._run = False
        self._worker_th: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._ratecv_state = None  # stato per audioop.ratecv

        if self.auto_start:
            self.start_listening()
        else:
            self._publish_status("idle")

    def _publish_status(self, s: str):
        msg = String()
        msg.data = s
        self.pub_status.publish(msg)

    def _on_system_cmd(self, msg):
        if msg.data.strip().lower() == "shutdown":
            self.get_logger().info("Shutdown richiesto: stop ascolto e chiusura.")
            self.stop_listening()   # la tua funzione esistente
            self.destroy_node()
            rclpy.shutdown()

    def _on_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if not cmd:
            return
        
        if cmd == "start":
            self.start_listening()
        elif cmd == "stop":
            self.stop_listening()
        elif cmd == "toggle":
            self.stop_listening() if self.is_listening() else self.start_listening()
        elif cmd == "status":
            self._publish_status("listening" if self.is_listening() else "idle")
        else:
            self.get_logger().warn(f"Comando sconosciuto su /tommy/voice/cmd: '{msg.data}'")

    def is_listening(self) -> bool:
        with self._lock:
            return self._run

    def start_listening(self):
        with self._lock:
            if self._run:
                self._publish_status("listening")
                return
            self._run = True

        self.get_logger().info("Avvio ascolto microfono (Vosk)...")
        self._drain_queue()
        self._ratecv_state = None

        # ~100ms: 48000/10 = 4800 frame (ok anche a 44100)
        blocksize = max(1024, int(self.input_sr / 10))

        def callback(indata, frames, time_info, status):
            if not self.is_listening():
                return
            try:
                self._audio_q.put_nowait(bytes(indata))
            except queue.Full:
                pass

        try:
            self._stream = sd.RawInputStream(
                samplerate=self.input_sr,
                blocksize=blocksize,
                device=None if self.device == -1 else self.device,
                dtype='int16',
                channels=1,
                callback=callback
            )
            self._stream.start()
        except Exception as e:
            with self._lock:
                self._run = False
            self.get_logger().error(f"Impossibile avviare lo stream audio: {e}")
            self._publish_status("error")
            return

        self._worker_th = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_th.start()
        self._publish_status("listening")

    def stop_listening(self):
        with self._lock:
            if not self._run:
                self._publish_status("idle")
                return
            self._run = False

        self.get_logger().info("Stop ascolto microfono.")
        try:
            if self._stream:
                self._stream.stop()
                self._stream.close()
        except Exception:
            pass
        self._stream = None
        self._publish_status("idle")

    def _drain_queue(self):
        try:
            while True:
                self._audio_q.get_nowait()
        except queue.Empty:
            pass

    def _resample_if_needed(self, pcm16_bytes: bytes) -> bytes:
        if not self.do_resample or self.input_sr == self.vosk_sr:
            return pcm16_bytes

        # mono S16_LE => width=2, channels=1
        converted, self._ratecv_state = audioop.ratecv(
            pcm16_bytes, 2, 1, self.input_sr, self.vosk_sr, self._ratecv_state
        )
        return converted

    def _worker_loop(self):
        last_partial = ""

        while self.is_listening():
            try:
                data = self._audio_q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                data = self._resample_if_needed(data)
            except Exception as e:
                self.get_logger().error(f"Errore resampling: {e}")
                continue

            try:
                is_final = self.rec.AcceptWaveform(data)
            except Exception as e:
                self.get_logger().error(f"Errore recognizer: {e}")
                continue

            if is_final:
                res = json.loads(self.rec.Result())
                text = (res.get("text") or "").strip()
                if len(text) >= self.min_text_len:
                    out = String()
                    out.data = text
                    self.pub_text.publish(out)
                    self.get_logger().info(f"STT: {text}")
                last_partial = ""
            else:
                if self.enable_partial and self.pub_partial:
                    pres = json.loads(self.rec.PartialResult())
                    p = (pres.get("partial") or "").strip()
                    if p and p != last_partial:
                        last_partial = p
                        out = String()
                        out.data = p
                        self.pub_partial.publish(out)

        time.sleep(0.05)


def main():
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop pulito stream audio
        try:
            node.stop_listening()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
