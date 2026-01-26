
#!/usr/bin/env python3

import os
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class SpeakNode(Node):
    """
    Nodo TTS (Text To Speech) basato su Piper.

    - Riceve testo da /robot/tts/say
    - Genera audio WAV con Piper
    - Riproduce audio con aplay
    - Pubblica stato su /robot/tts/state (speaking | idle)
    """

    def __init__(self):
        super().__init__('speak_node')

        # === Risoluzione path del modello Piper (ROS-correct) ===
        share_dir = get_package_share_directory('robot_head')
        model_dir = os.path.join(share_dir, 'models', 'piper')

        self.model_path = os.path.join(model_dir, 'it_IT-paola-medium.onnx')
        self.config_path = os.path.join(model_dir, 'it_IT-paola-medium.onnx.json')

        # === Verifica file ===
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Modello Piper non trovato: {self.model_path}")
        if not os.path.exists(self.config_path):
            self.get_logger().warning(f"Config Piper non trovata: {self.config_path}")

        # === ROS interfaces ===
        self.sub_say = self.create_subscription(
            String,
            '/tommy/speech/say',
            self.on_text,
            10
        )

        self.pub_state = self.create_publisher(
            String,
            '/tommy/voice/state',
            10
        )

        self.get_logger().info('SpeakNode avviato')
        self.get_logger().info(f"Model : {self.model_path}")
        self.get_logger().info(f"Config: {self.config_path}")

    # -------------------------------------------------------------

    def publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)

    # -------------------------------------------------------------

    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.publish_state('speaking')

        wav_path = None
        try:
            # File WAV temporaneo
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                wav_path = f.name

            # Comando Piper
            cmd = [
                'piper',
                '-m', self.model_path,
                '-f', wav_path
            ]

            if os.path.exists(self.config_path):
                cmd += ['-c', self.config_path]

            self.get_logger().info(f"TTS CMD: {' '.join(cmd)}")

            # Generazione audio
            subprocess.run(
                cmd,
                input=text,
                text=True,
                check=True
            )

            # Riproduzione audio
            subprocess.run(
                ['aplay', '-q', wav_path],
                check=True
            )

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Piper/aplay error: {e}")

        except Exception as e:
            self.get_logger().error(f"Errore TTS: {e}")

        finally:
            if wav_path and os.path.exists(wav_path):
                os.unlink(wav_path)

            self.publish_state('idle')


# ================================================================

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
