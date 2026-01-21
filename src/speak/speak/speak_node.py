#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
import queue
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeakNode(Node):
    """
    /talk        std_msgs/String  -> pronuncia testo (italiano) via Piper
    /talk/stop   std_msgs/String  -> interrompe riproduzione e svuota coda
    """

    def __init__(self):
        super().__init__('speak_node')

        # Modello italiano (soluzione definitiva offline)
        home = os.path.expanduser("~")
        self.model = os.path.join(
            home, ".local/share/piper/voices/it_IT-paola-medium/it_IT-paola-medium.onnx"
        )
        self.config = self.model + ".json"

        # Playback process (aplay)
        self._aplay_proc: subprocess.Popen | None = None

        # Coda per evitare sovrapposizioni
        self._q: "queue.Queue[str]" = queue.Queue()
        self._stop_event = threading.Event()

        self.create_subscription(String, 'talk', self._talk_cb, 10)
        self.create_subscription(String, 'talk/stop', self._stop_cb, 10)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"Speak (Piper) avviato. Topic: /talk | Modello: {self.model}"
        )

    def _talk_cb(self, msg: String):
        text = (msg.data or "").strip()
        if text:
            self._q.put(text)

    def _stop_cb(self, msg: String):
        self._flush_queue()
        self._stop_playback()

    def _flush_queue(self):
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass

    def _stop_playback(self):
        if self._aplay_proc and self._aplay_proc.poll() is None:
            try:
                self._aplay_proc.send_signal(signal.SIGTERM)
            except Exception:
                pass
        self._aplay_proc = None

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                text = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._speak_once(text)
            except Exception as e:
                self.get_logger().error(f"Errore Piper TTS: {e}")

    def _speak_once(self, text: str):
        self.get_logger().info(f'TTS: "{text}"')

        # Comando Piper (stdout)
        piper_cmd = [
            "piper",
            "--model", self.model,
            "--config", self.config,
            "--length_scale", "1.18",
            "--noise_scale", "0.55",
            "--noise_w", "0.65",
        ]

        # Avvio Piper
        piper_proc = subprocess.Popen(
            piper_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Avvio aplay che legge da stdin
        self._aplay_proc = subprocess.Popen(
            ["aplay", "-q"],
            stdin=piper_proc.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        try:
            # Invia testo a Piper
            assert piper_proc.stdin is not None
            piper_proc.stdin.write((text + "\n").encode("utf-8"))
            piper_proc.stdin.close()

            # Attendi fine riproduzione
            self._aplay_proc.wait()
        finally:
            # Cleanup sicuro
            try:
                piper_proc.terminate()
            except Exception:
                pass
            self._aplay_proc = None

        def destroy_node(self):
            self._stop_event.set()
            self._flush_queue()
            self._stop_playback()
            super().destroy_node()


def main():
    rclpy.init()
    node = SpeakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
