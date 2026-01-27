#!/usr/bin/env python3
import os
import time
import subprocess
import threading
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class SpeakNode(Node):
    """
    SpeakNode reattivo e affidabile:
    - Callback ROS non bloccante (enqueue)
    - Worker thread: ACK immediato (wav corto) + Piper CLI + aplay
    - Interrupt: termina aplay e rimpiazza con ultimo testo
    - WAV su /dev/shm per ridurre I/O
    """

    def __init__(self):
        super().__init__('speak_node')

        share_dir = get_package_share_directory('robot_head')
        model_dir = os.path.join(share_dir, 'models', 'piper')

        # Voce veloce (riccardo x_low)
        self.model_path = os.path.join(model_dir, 'it_IT-riccardo-x_low.onnx')
        self.config_path = os.path.join(model_dir, 'it_IT-riccardo-x_low.onnx.json')

        self.declare_parameter('interrupt', True)
        self.interrupt = bool(self.get_parameter('interrupt').value)

        self.declare_parameter('ack_text', 'Sì.')
        self.ack_text = str(self.get_parameter('ack_text').value).strip() or 'Sì.'

        self._wav_path = '/dev/shm/tommy_tts.wav' if os.path.isdir('/dev/shm') else '/tmp/tommy_tts.wav'
        self._ack_wav = '/dev/shm/tommy_ack.wav' if os.path.isdir('/dev/shm') else '/tmp/tommy_ack.wav'

        # ROS topics
        self.sub_say = self.create_subscription(String, '/tommy/speech/say', self.on_text, 10)
        self.pub_state = self.create_publisher(String, '/tommy/voice/state', 10)

        # Queue + worker
        self._q: "queue.Queue[tuple[str, float]]" = queue.Queue(maxsize=5)
        self._stop = threading.Event()

        # Per interrupt audio
        self._aplay_proc: subprocess.Popen | None = None
        self._proc_lock = threading.Lock()

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info("SpeakNode avviato (ACK + Piper CLI, reliable)")
        self.get_logger().info(f"Model : {self.model_path}")
        self.get_logger().info(f"Config: {self.config_path}")
        self.get_logger().info(f"WAV   : {self._wav_path}")
        self.get_logger().info(f"ACK   : {self._ack_wav} (text={self.ack_text!r})")
        self.get_logger().info(f"interrupt={self.interrupt}")

        # Warm-up: genera ack e fa una synth breve (stabilizza latenza)
        self._ensure_ack()
        self._warmup()

    def publish_state(self, state: str):
        m = String()
        m.data = state
        self.pub_state.publish(m)

    def _interrupt_current_audio(self):
        with self._proc_lock:
            if self._aplay_proc and self._aplay_proc.poll() is None:
                try:
                    self._aplay_proc.terminate()
                except Exception:
                    pass

    def _drain_queue(self):
        try:
            while True:
                self._q.get_nowait()
                self._q.task_done()
        except queue.Empty:
            pass

    def _piper_to_wav(self, text: str, out_wav: str):
        cmd = ['piper', '-m', self.model_path, '-f', out_wav]
        if os.path.exists(self.config_path):
            cmd += ['-c', self.config_path]
        subprocess.run(
            cmd,
            input=text,
            text=True,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def _play_wav(self, wav_path: str):
        with self._proc_lock:
            self._aplay_proc = subprocess.Popen(
                ['aplay', '-q', wav_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            proc = self._aplay_proc
        proc.wait()
        with self._proc_lock:
            self._aplay_proc = None

    def _ensure_ack(self):
        if os.path.exists(self._ack_wav):
            return
        try:
            self._piper_to_wav(self.ack_text, self._ack_wav)
            self.get_logger().info("ACK generato: OK")
        except Exception as e:
            self.get_logger().warning(f"ACK non generato: {e}")

    def _warmup(self):
        try:
            self._piper_to_wav("Ciao.", self._wav_path)
            self.get_logger().info("Warm-up Piper: OK")
        except Exception as e:
            self.get_logger().warning(f"Warm-up Piper fallito (non bloccante): {e}")

    # -------- ROS callback
    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        t0 = time.perf_counter()

        if self.interrupt:
            self._interrupt_current_audio()
            self._drain_queue()

        try:
            self._q.put_nowait((text, t0))
        except queue.Full:
            # keep newest
            try:
                _ = self._q.get_nowait()
                self._q.task_done()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait((text, t0))
            except queue.Full:
                pass

    # -------- Worker
    def _worker_loop(self):
        while not self._stop.is_set():
            try:
                text, t0 = self._q.get(timeout=0.1)
            except queue.Empty:
                continue

            self.publish_state("speaking")

            try:
                # 1) ACK immediato (se disponibile)
                t_ack_start = time.perf_counter()
                if os.path.exists(self._ack_wav):
                    try:
                        self._play_wav(self._ack_wav)
                    except Exception:
                        pass
                t_ack_end = time.perf_counter()

                # Se è arrivato qualcosa di nuovo, non sprecare tempo a sintetizzare il vecchio
                if self.interrupt and not self._q.empty():
                    self.get_logger().info("Nuovo testo in coda: skip synth frase corrente")
                    continue

                # 2) Piper synth frase vera
                t_gen0 = time.perf_counter()
                self._piper_to_wav(text, self._wav_path)
                t_gen1 = time.perf_counter()

                # 3) Playback frase vera
                t_play0 = time.perf_counter()
                self._play_wav(self._wav_path)
                t_play1 = time.perf_counter()

                self.get_logger().info(
                    f"TTS timing: ack={(t_ack_start - t0)*1000:.0f}ms | "
                    f"ack_play={(t_ack_end - t_ack_start)*1000:.0f}ms | "
                    f"gen={(t_gen1 - t_gen0)*1000:.0f}ms | "
                    f"total={(t_play1 - t0)*1000:.0f}ms"
                )

            except Exception as e:
                self.get_logger().error(f"Errore TTS: {e}")
            finally:
                self.publish_state("idle")
                self._q.task_done()

    def destroy_node(self):
        self._stop.set()
        self._interrupt_current_audio()
        super().destroy_node()


def main():
    rclpy.init()
    node = SpeakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
