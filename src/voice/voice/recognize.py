#!/usr/bin/env python3
import json
import queue
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
from vosk import Model, KaldiRecognizer

from ament_index_python.packages import get_package_share_directory


class VoskRecognizerNode(Node):
    def __init__(self):
        super().__init__('recognize')

        # Parametri
        self.declare_parameter('model_subdir', 'models/vosk-model-small-it-0.22')
        self.declare_parameter('device_index', 2)    # PortAudio index: dal tuo elenco è il mic hw:3,0
        self.declare_parameter('samplerate', 44100)  # hw:3,0 supporta 44100/48000
        self.declare_parameter('channels', 1)

        model_subdir = self.get_parameter('model_subdir').value
        self.device_index = int(self.get_parameter('device_index').value)
        self.samplerate = int(self.get_parameter('samplerate').value)
        self.channels = int(self.get_parameter('channels').value)

        # Modello Vosk dentro share del pacchetto
        share_dir = Path(get_package_share_directory('voice'))
        model_dir = (share_dir / model_subdir).resolve()
        if not model_dir.exists():
            raise RuntimeError(
                f"Modello Vosk non trovato in: {model_dir}\n"
                "Copia il modello in voice/models/... e ricompila (colcon build)."
            )

        self.get_logger().info(f"Carico modello Vosk da: {model_dir}")
        self.model = Model(str(model_dir))
        self.rec = KaldiRecognizer(self.model, self.samplerate)
        self.rec.SetWords(False)

        # Verifica device
        d = sd.query_devices(self.device_index)
        if d.get('max_input_channels', 0) <= 0:
            raise RuntimeError(f"Il device_index={self.device_index} non è un input device: {d}")
        self.get_logger().info(f"Uso input device #{self.device_index}: {d.get('name')}")

        self.pub = self.create_publisher(String, 'voice', 10)
        self.q = queue.Queue(maxsize=50)

        self.stream = sd.RawInputStream(
            samplerate=self.samplerate,
            blocksize=8000,
            device=self.device_index,
            dtype='int16',
            channels=self.channels,
            callback=self._audio_callback,
        )
        self.stream.start()

        self.get_logger().info(
            f"Ascolto @ {self.samplerate} Hz, {self.channels}ch. Pubblico frasi finali su /voice"
        )

        self.timer = self.create_timer(0.01, self._process_audio)

    def _audio_callback(self, indata, frames, time, status):
        try:
            self.q.put_nowait(bytes(indata))
        except queue.Full:
            pass

    def _process_audio(self):
        try:
            data = self.q.get_nowait()
        except queue.Empty:
            return

        if self.rec.AcceptWaveform(data):
            result = json.loads(self.rec.Result())
            text = (result.get("text") or "").strip()
            if text:
                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"VOICE: {text}")

    def destroy_node(self):
        try:
            if getattr(self, 'stream', None):
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = VoskRecognizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
