#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class HeadOrchestrator(Node):
    VALID_STATES = ("idle", "observe", "listen", "speak", "demo","exit","spegniti","spegni")

    def __init__(self):
        super().__init__("head_orchestrator")

        # ---------------- Params ----------------
        self.declare_parameter("eyes_cmd_topic", "/robot_head/eyes/cmd")
        self.declare_parameter("mode_topic", "/robot_head/state")
        self.declare_parameter("listen_resume_delay_ms", 250)
        self.declare_parameter("default_state", "idle")

        # Topics (allineati al tuo speak_node)
        self.declare_parameter("speech_say_topic", "/tommy/speech/say")
        self.declare_parameter("tts_state_topic", "/tommy/voice/state")
        self.declare_parameter("voice_cmd_topic", "/tommy/voice/cmd")
        self.declare_parameter("vision_enable_topic", "/tommy/vision/enable")

        self.eyes_cmd_topic = str(self.get_parameter("eyes_cmd_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.listen_resume_delay_ms = int(self.get_parameter("listen_resume_delay_ms").value)

        self.speech_say_topic = str(self.get_parameter("speech_say_topic").value)
        self.tts_state_topic = str(self.get_parameter("tts_state_topic").value)
        self.voice_cmd_topic = str(self.get_parameter("voice_cmd_topic").value)
        self.vision_enable_topic = str(self.get_parameter("vision_enable_topic").value)

        default_state = str(self.get_parameter("default_state").value).strip().lower()
        if default_state not in self.VALID_STATES:
            default_state = "idle"

        # ---------------- Internal state ----------------
        self.current_state = default_state
        self.current_mode = "idle"
        self.tts_speaking = False
        self.want_listen = False
        self._resume_timer = None

        # ---------------- ROS interfaces ----------------
        self.cmd_sub = self.create_subscription(String, "/head/cmd", self.cmd_callback, 10)
        self.sys_cmd_pub = self.create_publisher(String, "/robot_head/system/cmd", 10)

        self.state_pub = self.create_publisher(String, "/head/state", 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)

        self.voice_cmd_pub = self.create_publisher(String, self.voice_cmd_topic, 10)
        self.vision_enable_pub = self.create_publisher(Bool, self.vision_enable_topic, 10)

        self.eyes_cmd_pub = self.create_publisher(String, self.eyes_cmd_topic, 10)

        # Publisher verso speak_node
        self.speech_say_pub = self.create_publisher(String, self.speech_say_topic, 10)

        # Feedback da speak_node (speaking|idle)
        self.tts_state_sub = self.create_subscription(String, self.tts_state_topic, self.on_tts_state, 10)

        self.timer = self.create_timer(2.0, self.publish_state)

        # Apply initial
        self.publish_state()
        self.apply_policy(self.current_state)

        self.get_logger().info(
            f"HeadOrchestrator avviato (state={self.current_state}) "
            f"speech_say_topic={self.speech_say_topic} tts_state_topic={self.tts_state_topic}"
        )

    # ---------------- Publish helpers ----------------
    def publish_state(self):
        self.state_pub.publish(String(data=self.current_state))

    def _pub_mode(self, mode: str):
        self.mode_pub.publish(String(data=f"mode={mode}"))

    def _pub_voice_cmd(self, cmd: str):
        self.voice_cmd_pub.publish(String(data=cmd))

    def _pub_vision_enable(self, enable: bool):
        self.vision_enable_pub.publish(Bool(data=bool(enable)))

    def _pub_eyes_cmd(self, s: str):
        self.eyes_cmd_pub.publish(String(data=s))

    def say(self, text: str):
        text = (text or "").strip()
        if not text:
            return
        # facoltativo: mostra “thinking” finché non parte speaking
        if not self.tts_speaking:
            self.apply_mode("thinking")
        self.speech_say_pub.publish(String(data=text))

    # ---------------- Mode table ----------------
    def apply_mode(self, mode: str, pupil_norm: tuple[float, float] | None = None):
        mode = (mode or "idle").strip().lower()

        eyes_state = "normal"
        blink = 1
        pupil_cmd = "pupil=center"
        listen = 1

        if mode == "idle":
            eyes_state, blink, listen = "normal", 1, 0

        elif mode == "listening":
            eyes_state, blink, listen = "alert", 1, 1

        elif mode == "thinking":
            eyes_state, blink, listen = "attentive", 0, 0

        elif mode == "speaking":
            eyes_state, blink, listen = "attentive", 0, 0

        elif mode == "tracking":
            eyes_state, blink, listen = "attentive", 1, 1
            if pupil_norm is not None:
                nx, ny = pupil_norm
                nx = max(-1.0, min(1.0, float(nx)))
                ny = max(-1.0, min(1.0, float(ny)))
                pupil_cmd = f"pupil={nx:.2f},{ny:.2f}"

        elif mode == "sleep":
            eyes_state, blink, listen = "sleep", 0, 0

        elif mode == "error":
            eyes_state, blink, listen = "alert", 0, 0

        self._pub_eyes_cmd(f"state={eyes_state}")
        self._pub_eyes_cmd(f"blink={blink}")
        self._pub_eyes_cmd(pupil_cmd)

        # Compatibilità con voice_node attuale: start/stop
        self._pub_voice_cmd("start" if listen == 1 else "stop")

        self.current_mode = mode
        self._pub_mode(mode)

    # ---------------- High-level policy ----------------
    def apply_policy(self, state: str):
        state = (state or "idle").strip().lower()

        if state == "idle":
            self.want_listen = False
            self._pub_vision_enable(False)
            self.apply_mode("speaking" if self.tts_speaking else "idle")
            if not self.tts_speaking:
                self._pub_voice_cmd("stop")
            return

        if state == "observe":
            self.want_listen = False
            self._pub_vision_enable(True)
            self.apply_mode("speaking" if self.tts_speaking else "idle")
            if not self.tts_speaking:
                self._pub_voice_cmd("stop")
                self._pub_mode("observe")
            return

        if state == "listen":
            self.want_listen = True
            self._pub_vision_enable(True)
            self.apply_mode("speaking" if self.tts_speaking else "listening")
            return

        if state == "speak":
            self.want_listen = False
            self._pub_vision_enable(True)
            # In speak: non ascoltare
            self._pub_voice_cmd("stop")
            self.apply_mode("speaking" if self.tts_speaking else "thinking")
            return

        if state == "demo":
            self.want_listen = True
            self._pub_vision_enable(True)
            self.apply_mode("speaking" if self.tts_speaking else "listening")
            return

    # ---------------- Callbacks ----------------
    def cmd_callback(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd not in self.VALID_STATES:
            self.get_logger().warn(f"Comando non valido su /head/cmd: '{msg.data}'")
            return
        if cmd in ("exit", "shutdown", "spegni"):
            self.get_logger().info("Comando EXIT: spegnimento coordinato.")

            # 1) occhi: animazione e stop
            self._pub_eyes_cmd("state=stop")

            # 2) rilascia camera + chiudi nodi
            self.sys_cmd_pub.publish(String(data="shutdown"))

            # 3) safety: stop voice subito (se vuoi immediato)
            self._pub_voice_cmd("stop")

            # 4) dai tempo all’animazione occhi (opzionale ma consigliato)
            def _finalize():
                self.get_logger().info("Chiusura orchestrator.")
                self.destroy_node()
                rclpy.shutdown()

            # one-shot timer (es. 0.9s) per far finire animazione
            self.create_timer(0.9, _finalize)
            return
        self.current_state = cmd
        self.get_logger().info(f"Nuovo stato: {self.current_state}")
        self.publish_state()
        self.apply_policy(self.current_state)

    def on_tts_state(self, msg: String):
        # speak_node pubblica: 'speaking' o 'idle' :contentReference[oaicite:1]{index=1}
        s = (msg.data or "").strip().lower()
        if s not in ("speaking", "idle"):
            return

        was = self.tts_speaking
        self.tts_speaking = (s == "speaking")

        if self.tts_speaking and not was:
            # Priorità: speaking blocca ascolto
            self.get_logger().info("TTS=speaking -> mode=speaking")
            self._cancel_resume_timer()
            self.apply_mode("speaking")
            return

        if (not self.tts_speaking) and was:
            # Fine speaking -> ripristina, con delay se vogliamo ascoltare
            self.get_logger().info("TTS=idle -> ripristino")
            if self.want_listen:
                self._schedule_resume_listen()
            else:
                self.apply_policy(self.current_state)

    # ---------------- Resume timer ----------------
    def _cancel_resume_timer(self):
        if self._resume_timer is not None:
            try:
                self._resume_timer.cancel()
            except Exception:
                pass
            self._resume_timer = None

    def _schedule_resume_listen(self):
        self._cancel_resume_timer()
        delay_s = max(0.0, self.listen_resume_delay_ms / 1000.0)

        def _cb():
            self._cancel_resume_timer()
            if self.tts_speaking:
                return
            self.apply_policy(self.current_state)

        self._resume_timer = self.create_timer(delay_s, _cb)


def main(args=None):
    rclpy.init(args=args)
    node = HeadOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
