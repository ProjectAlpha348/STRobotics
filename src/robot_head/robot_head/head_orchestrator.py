
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class HeadOrchestrator(Node):
    """
    Orchestratore testa "Tommy".

    High-level:
      - /head/cmd   : idle|observe|listen|speak|demo
      - /head/state : stato high-level corrente

    Controllo nodi:
      - /tommy/vision/enable : Bool
      - /tommy/voice/cmd     : String (start|stop)  <-- compatibilità con voice_node attuale
      - /robot_head/eyes/cmd : String (state=..., blink=..., pupil=...)

    Feedback:
      - /tommy/voice/state   : String (speaking|idle)  <-- pubblicato da SpeakNode (TTS)
      - /tommy/voice/status  : String (opzionale, solo debug)

    Principio:
      - quando TTS parla (speaking) => voice stop + eyes attentive + blink=0
      - quando TTS finisce (idle)   => se lo stato lo richiede => voice start (dopo delay) + eyes alert/normal
    """

    VALID_STATES = ("idle", "observe", "listen", "speak", "demo")

    def __init__(self):
        super().__init__("head_orchestrator")

        # ---------------- Params ----------------
        self.declare_parameter("eyes_cmd_topic", "/robot_head/eyes/cmd")
        self.declare_parameter("mode_topic", "/robot_head/state")
        self.declare_parameter("listen_resume_delay_ms", 250)  # anti-rientro audio
        self.declare_parameter("default_state", "idle")

        self.eyes_cmd_topic = str(self.get_parameter("eyes_cmd_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.listen_resume_delay_ms = int(self.get_parameter("listen_resume_delay_ms").value)

        default_state = str(self.get_parameter("default_state").value).strip().lower()
        if default_state not in self.VALID_STATES:
            default_state = "idle"

        # ---------------- Internal state ----------------
        self.current_state = default_state      # high-level
        self.current_mode = "idle"              # mode-level (tabella)
        self.tts_speaking = False
        self.want_listen = False                # desiderio di ascolto in base allo stato (listen/demo)

        self._resume_timer = None               # timer one-shot per riattivare ascolto

        # ---------------- ROS interfaces ----------------
        # High-level commands
        self.cmd_sub = self.create_subscription(String, "/head/cmd", self.cmd_callback, 10)

        # State publication (high-level)
        self.state_pub = self.create_publisher(String, "/head/state", 10)

        # Mode publication (mode-level)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)

        # Control: voice + vision
        self.voice_cmd_pub = self.create_publisher(String, "/tommy/voice/cmd", 10)
        self.vision_enable_pub = self.create_publisher(Bool, "/tommy/vision/enable", 10)

        # Control: eyes
        self.eyes_cmd_pub = self.create_publisher(String, self.eyes_cmd_topic, 10)

        # Feedback: TTS state (published by SpeakNode)
        self.tts_state_sub = self.create_subscription(String, "/tommy/voice/state", self.on_tts_state, 10)

        # (Optional) feedback STT status (published by VoiceNode)
        self.voice_status_sub = self.create_subscription(String, "/tommy/voice/status", self.on_voice_status, 10)

        # Heartbeat publish
        self.timer = self.create_timer(2.0, self.publish_state)

        # Apply initial policy
        self.publish_state()
        self.apply_policy(self.current_state)

        self.get_logger().info(
            f"HeadOrchestrator avviato (state={self.current_state}) "
            f"eyes_cmd_topic={self.eyes_cmd_topic} listen_resume_delay_ms={self.listen_resume_delay_ms}"
        )

    # ---------------- Helpers publish ----------------
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

    # ---------------- Mode table (definitiva) ----------------
    def apply_mode(self, mode: str, pupil_norm: tuple[float, float] | None = None):
        """
        mode: idle, listening, thinking, speaking, tracking, sleep, error
        pupil_norm: (nx, ny) in [-1..+1] usato solo in tracking
        """

        mode = (mode or "idle").strip().lower()

        # Default
        eyes_state = "normal"
        blink = 1
        pupil_cmd = "pupil=center"
        listen = 1

        if mode == "idle":
            eyes_state, blink, listen = "normal", 1, 1

        elif mode == "listening":
            eyes_state, blink, listen = "alert", 1, 1
            pupil_cmd = "pupil=center"

        elif mode == "thinking":
            eyes_state, blink, listen = "attentive", 0, 1
            pupil_cmd = "pupil=center"

        elif mode == "speaking":
            eyes_state, blink, listen = "attentive", 0, 0
            pupil_cmd = "pupil=center"

        elif mode == "tracking":
            eyes_state, blink, listen = "attentive", 1, 1
            if pupil_norm is not None:
                nx, ny = pupil_norm
                nx = max(-1.0, min(1.0, float(nx)))
                ny = max(-1.0, min(1.0, float(ny)))
                pupil_cmd = f"pupil={nx:.2f},{ny:.2f}"
            else:
                pupil_cmd = "pupil=center"

        elif mode == "sleep":
            eyes_state, blink, listen = "sleep", 0, 0
            pupil_cmd = "pupil=center"

        elif mode == "error":
            eyes_state, blink, listen = "alert", 0, 0
            pupil_cmd = "pupil=center"

        # Pubblica occhi (semplice e robusto: 3 messaggi separati)
        self._pub_eyes_cmd(f"state={eyes_state}")
        self._pub_eyes_cmd(f"blink={blink}")
        self._pub_eyes_cmd(pupil_cmd)

        # Pubblica voce (compatibilità: start/stop)
        self._pub_voice_cmd("start" if listen == 1 else "stop")

        # Pubblica mode (debug/integrabilità)
        self.current_mode = mode
        self._pub_mode(mode)

    # ---------------- High-level policy ----------------
    def apply_policy(self, state: str):
        """
        Mappa stato high-level -> obiettivi (vision, want_listen) e imposta il mode base.
        Il mode finale può essere forzato da TTS speaking.
        """
        state = (state or "idle").strip().lower()

        if state == "idle":
            self.want_listen = False
            self._pub_vision_enable(False)
            # Se non stiamo parlando, mettiamo idle “pulito”
            if not self.tts_speaking:
                self.apply_mode("idle")
            else:
                self.apply_mode("speaking")
            return

        if state == "observe":
            self.want_listen = False
            self._pub_vision_enable(True)
            if not self.tts_speaking:
                self.apply_mode("idle")  # occhi normal, ascolto stop (ma qui il mode idle sarebbe listen=1; non vogliamo ascolto)
                # Forza stop ascolto perché observe deve essere voice OFF:
                self._pub_voice_cmd("stop")
                self._pub_mode("observe")
            else:
                self.apply_mode("speaking")
            return

        if state == "listen":
            self._pub_vision_enable(True)
            self.want_listen = True
            # se TTS parla, prevale speaking; altrimenti listening
            if not self.tts_speaking:
                self.apply_mode("listening")
            else:
                self.apply_mode("speaking")
            return

        if state == "speak":
            self._pub_vision_enable(True)
            self.want_listen = False
            # In speak: ascolto off comunque
            self.apply_mode("speaking" if self.tts_speaking else "thinking")
            # e forziamo stop in ogni caso
            self._pub_voice_cmd("stop")
            return

        if state == "demo":
            self._pub_vision_enable(True)
            self.want_listen = True
            if not self.tts_speaking:
                self.apply_mode("listening")
            else:
                self.apply_mode("speaking")
            return

    # ---------------- Callbacks ----------------
    def cmd_callback(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd not in self.VALID_STATES:
            self.get_logger().warn(f"Comando non valido su /head/cmd: '{msg.data}'")
            return

        if cmd == self.current_state:
            # re-apply policy (utile se un nodo è stato riavviato)
            self.get_logger().info(f"Stato già attivo: {cmd} (re-apply policy)")
            self.apply_policy(cmd)
            self.publish_state()
            return

        self.current_state = cmd
        self.get_logger().info(f"Nuovo stato impostato: {self.current_state}")
        self.publish_state()
        self.apply_policy(self.current_state)

    def on_tts_state(self, msg: String):
        """
        Topic: /tommy/voice/state
        Attesi: 'speaking' | 'idle'
        """
        s = (msg.data or "").strip().lower()
        if s not in ("speaking", "idle"):
            return

        was_speaking = self.tts_speaking
        self.tts_speaking = (s == "speaking")

        # TTS START: entra in speaking immediatamente (priorità massima)
        if self.tts_speaking and not was_speaking:
            self.get_logger().info("TTS=speaking -> mode=speaking + stop ascolto.")
            self._cancel_resume_timer()
            self.apply_mode("speaking")
            return

        # TTS END: ripristina in base allo stato high-level
        if (not self.tts_speaking) and was_speaking:
            self.get_logger().info("TTS=idle -> valuto ripristino ascolto/stato.")

            # anti-rientro: delay prima di start ascolto
            if self.want_listen:
                self._schedule_resume_listen()
            else:
                # Non vogliamo ascoltare: applichiamo la policy corrente senza listen
                self.apply_policy(self.current_state)

    def on_voice_status(self, msg: String):
        s = (msg.data or "").strip().lower()
        if s:
            self.get_logger().debug(f"Voice status: {s}")

    # ---------------- Resume timer helpers ----------------
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
            # one-shot
            self._cancel_resume_timer()
            # Se nel frattempo è ripartito speaking, non fare nulla
            if self.tts_speaking:
                return
            # Torna allo stato corrente (tipicamente listen/demo)
            self.apply_policy(self.current_state)

        # create_timer è periodico; lo cancelliamo nel callback per farlo one-shot
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
