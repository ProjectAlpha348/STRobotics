#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class HeadOrchestrator(Node):
    VALID_STATES = ("idle", "observe", "listen", "speak", "demo", "exit", "spegniti", "spegni")

    def __init__(self):
        super().__init__("head_orchestrator")

        # ---------------- Params ----------------
        self.declare_parameter("eyes_cmd_topic", "/robot_head/eyes/cmd")
        self.declare_parameter("mode_topic", "/robot_head/state")
        self.declare_parameter("listen_resume_delay_ms", 250)
        self.declare_parameter("default_state", "idle")

        # Speak (allineati al tuo speak_node)
        self.declare_parameter("speech_say_topic", "/tommy/speech/say")

        # FIX: nel tuo file era /tommy/voice/state (non coerente con la tua architettura)
        # Qui metto un default più sensato. Se il tuo speak_node usa un altro topic, puoi sovrascriverlo da launch.
        self.declare_parameter("tts_state_topic", "/robot/tts/state")

        # Voice control
        self.declare_parameter("voice_cmd_topic", "/tommy/voice/cmd")
        self.declare_parameter("voice_text_topic", "/tommy/voice/text")

        # Sensori ambiente (String: "temp=23.4;hum=48.2")
        self.declare_parameter("env_topic", "/tommy/sensors/env")
        self.declare_parameter("env_stale_timeout_s", 30.0)

        # Fallback: se non arriva mai lo stato TTS, riavvia ascolto dopo N secondi
        self.declare_parameter("tts_fallback_resume_s", 20.0)

        # Vision
        self.declare_parameter("vision_enable_topic", "/tommy/vision/enable")
        self.declare_parameter("vision_state_topic", "/tommy/vision/state")
        self.declare_parameter("vision_idle_value", "idle")
        self.declare_parameter("vision_idle_timeout_s", 5.0)

        # -------- Load params --------
        voice_text_topic = str(self.get_parameter("voice_text_topic").value)

        self.eyes_cmd_topic = str(self.get_parameter("eyes_cmd_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.listen_resume_delay_ms = int(self.get_parameter("listen_resume_delay_ms").value)

        self.speech_say_topic = str(self.get_parameter("speech_say_topic").value)
        self.tts_state_topic = str(self.get_parameter("tts_state_topic").value)
        self.voice_cmd_topic = str(self.get_parameter("voice_cmd_topic").value)

        self.env_topic = str(self.get_parameter("env_topic").value)
        self.env_stale_timeout_s = float(self.get_parameter("env_stale_timeout_s").value)
        self.tts_fallback_resume_s = float(self.get_parameter("tts_fallback_resume_s").value)

        self.vision_enable_topic = str(self.get_parameter("vision_enable_topic").value)
        self.vision_state_topic = str(self.get_parameter("vision_state_topic").value)
        self.vision_idle_value = str(self.get_parameter("vision_idle_value").value).strip().lower()
        self.vision_idle_timeout_s = float(self.get_parameter("vision_idle_timeout_s").value)

        default_state = str(self.get_parameter("default_state").value).strip().lower()
        if default_state not in self.VALID_STATES:
            default_state = "idle"

        # ---------------- Internal state ----------------
        self.current_state = default_state
        self.current_mode = "idle"
        self.tts_speaking = False
        self.want_listen = False
        self._resume_timer = None

        # Cache sensori
        self.last_temp = None
        self.last_hum = None
        self.last_env_ts = 0.0

        # Vision state cache
        self.vision_state = self.vision_idle_value

        # Exit orchestration
        self.exit_requested = False
        self._exit_lock = threading.Lock()
        self._exit_timer = None

        # Fallback timer TTS
        self._tts_fallback_timer = None
        self._tts_fallback_armed = False

        # ---------------- ROS interfaces ----------------
        self.cmd_sub = self.create_subscription(String, "/head/cmd", self.cmd_callback, 10)
        self.sys_cmd_pub = self.create_publisher(String, "/robot_head/system/cmd", 10)

        self.state_pub = self.create_publisher(String, "/head/state", 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)

        # Questo topic viene usato anche come controllo voice_node (start/stop)
        self.voice_cmd_pub = self.create_publisher(String, self.voice_cmd_topic, 10)

        # Comandi vocali dal testo riconosciuto (STT)
        self.voice_text_sub = self.create_subscription(String, voice_text_topic, self.on_voice_cmd, 10)

        # Sensori ambiente
        self.env_sub = self.create_subscription(String, self.env_topic, self.on_env, 10)

        # Vision
        self.vision_enable_pub = self.create_publisher(Bool, self.vision_enable_topic, 10)
        self.vision_state_sub = self.create_subscription(String, self.vision_state_topic, self.on_vision_state, 10)

        # Eyes
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
            f"speech_say_topic={self.speech_say_topic} tts_state_topic={self.tts_state_topic} "
            f"voice_text_topic={voice_text_topic} env_topic={self.env_topic} "
            f"vision_enable_topic={self.vision_enable_topic} vision_state_topic={self.vision_state_topic}"
        )

    # ---------------- Small helpers ----------------
    @staticmethod
    def _norm(s: str) -> str:
        return (s or "").strip().lower()

    def _one_shot(self, delay_s: float, cb):
        """
        rclpy Timer è periodico: per fare "one-shot" lo cancelliamo dentro la callback.
        """
        if self._exit_timer is not None:
            try:
                self._exit_timer.cancel()
            except Exception:
                pass
            self._exit_timer = None

        def _wrapped():
            try:
                cb()
            finally:
                if self._exit_timer is not None:
                    try:
                        self._exit_timer.cancel()
                    except Exception:
                        pass
                    self._exit_timer = None

        self._exit_timer = self.create_timer(max(0.0, float(delay_s)), _wrapped)

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

    # ---------------- TTS fallback resume ----------------
    def _cancel_tts_fallback(self):
        if self._tts_fallback_timer is not None:
            try:
                self._tts_fallback_timer.cancel()
            except Exception:
                pass
            self._tts_fallback_timer = None
        self._tts_fallback_armed = False

    def _arm_tts_fallback(self):
        """
        Se il topic di stato TTS non arriva (o arriva tardi), questo timer riattiva l'ascolto.
        """
        self._cancel_tts_fallback()
        if self.tts_fallback_resume_s <= 0.0:
            return

        self._tts_fallback_armed = True

        def _cb():
            # one-shot
            self._cancel_tts_fallback()

            # Se vogliamo ascoltare (listen/demo) allora forziamo un resume
            if self.want_listen and (not self.tts_speaking):
                self.get_logger().warn("TTS fallback: riattivo ascolto (nessun idle ricevuto)")
                self.apply_mode("listening")

        self._tts_fallback_timer = self.create_timer(float(self.tts_fallback_resume_s), _cb)

    def say(self, text: str):
        text = (text or "").strip()
        if not text:
            return

        # entra in thinking (che stoppa il mic) e arma fallback
        self.apply_mode("thinking")
        self._arm_tts_fallback()

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
            self._pub_voice_cmd("stop")
            self.apply_mode("speaking" if self.tts_speaking else "thinking")
            return

        if state == "demo":
            self.want_listen = True
            self._pub_vision_enable(True)
            self.apply_mode("speaking" if self.tts_speaking else "listening")
            return

    # ---------------- Sensors ----------------
    def on_env(self, msg: String):
        """
        Atteso: "temp=23.4;hum=48.2"
        """
        s = (msg.data or "").strip()
        if not s:
            return
        try:
            parts = dict(x.split("=", 1) for x in s.split(";") if "=" in x)
            t = float(parts.get("temp"))
            h = float(parts.get("hum"))
            self.last_temp = t
            self.last_hum = h
            self.last_env_ts = time.time()
        except Exception:
            # niente spam: se formato errato lo ignoriamo
            pass

    def _env_is_fresh(self) -> bool:
        if self.last_temp is None or self.last_hum is None:
            return False
        return (time.time() - float(self.last_env_ts or 0.0)) <= self.env_stale_timeout_s

    def _handle_temp_request(self):
        # se siamo in listen/demo, vogliamo tornare ad ascoltare dopo la frase
        # (want_listen già True in quei mode)
        if not self._env_is_fresh():
            self.say("Non ho ancora i dati dei sensori. Aspetta un attimo e riprova.")
            return

        t = float(self.last_temp)
        h = float(self.last_hum)
        self.say(f"La temperatura è di {t:.1f} gradi, con un'umidità del {h:.0f} per cento.")

    # ---------------- Voice commands (UPDATED) ----------------
    def on_voice_cmd(self, msg: String):
        """
        Comandi vocali:
          - guarda          -> vision_enable True
          - stoppa visione  -> vision_enable False
          - uscita          -> shutdown coordinato (attesa vision idle)
          - temp/temperatura/umidità -> risposta vocale con T e U
        """
        cmd = self._norm(msg.data)
        if not cmd:
            return

        # Ignora i comandi tecnici che noi stessi pubblichiamo per il voice_node
        if cmd in ("start", "stop", "toggle", "status"):
            return

        if cmd == "guarda":
            self.get_logger().info("Vocale: 'guarda' -> abilito visione")
            self._pub_vision_enable(True)
            return

        if cmd == "stoppa visione":
            self.get_logger().info("Vocale: 'stoppa visione' -> disabilito visione")
            self._pub_vision_enable(False)
            return

        if cmd == "uscita":
            self.get_logger().info("Vocale: 'uscita' -> spegnimento coordinato")
            self._request_exit(source="voice")
            return

        # TEMP (match semplice ma efficace: include "temperatura" dai tuoi log)
        if cmd in ("temp", "temperatura", "umidità", "umidita"):
            self.get_logger().info("Vocale: 'temp/temperatura' -> rispondo con T e U")
            # assicura che il fallback ripristini ascolto: ha senso solo se sei in listen/demo
            self._handle_temp_request()
            return

        # altri comandi vocali ignorati
        return

    # ---------------- Vision state ----------------
    def on_vision_state(self, msg: String):
        self.vision_state = self._norm(msg.data) or self.vision_state

    # ---------------- Coordinated shutdown ----------------
    def _request_exit(self, source: str = "cmd"):
        with self._exit_lock:
            if self.exit_requested:
                self.get_logger().warn("Uscita già richiesta, ignoro.")
                return
            self.exit_requested = True

        threading.Thread(target=self._graceful_shutdown, args=(source,), daemon=True).start()

    def _graceful_shutdown(self, source: str):
        self.get_logger().info(f"EXIT: avvio sequenza (source={source})")

        # 1) Occhi: animazione e stop
        self._pub_eyes_cmd("state=stop")

        # 2) Stop ascolto subito
        self._pub_voice_cmd("stop")

        # 3) Disabilita visione e ATTENDI idle prima di shutdown sistema
        self._pub_vision_enable(False)

        t0 = time.time()
        while self.vision_state != self.vision_idle_value:
            if (time.time() - t0) > self.vision_idle_timeout_s:
                self.get_logger().warn(
                    f"EXIT: timeout attesa vision idle ({self.vision_idle_timeout_s:.1f}s). "
                    f"Ultimo state='{self.vision_state}'. Proseguo comunque."
                )
                break
            time.sleep(0.1)

        if self.vision_state == self.vision_idle_value:
            self.get_logger().info("EXIT: vision idle confermato")

        # 4) Shutdown coordinato
        self.sys_cmd_pub.publish(String(data="shutdown"))

        # 5) Dai tempo all’animazione occhi poi chiudi orchestrator
        def _finalize():
            self.get_logger().info("EXIT: chiusura orchestrator.")
            try:
                self.destroy_node()
            finally:
                rclpy.shutdown()

        self._one_shot(0.9, _finalize)

    # ---------------- Callbacks ----------------
    def cmd_callback(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd not in self.VALID_STATES:
            self.get_logger().warn(f"Comando non valido su /head/cmd: '{msg.data}'")
            return

        if cmd in ("exit", "shutdown", "spegni", "spegniti"):
            self.get_logger().info("Comando EXIT: spegnimento coordinato (attesa vision idle).")
            self._request_exit(source="/head/cmd")
            return

        self.current_state = cmd
        self.get_logger().info(f"Nuovo stato: {self.current_state}")
        self.publish_state()
        self.apply_policy(self.current_state)

    def on_tts_state(self, msg: String):
        """
        speak_node dovrebbe pubblicare: 'speaking' o 'idle'
        """
        s = (msg.data or "").strip().lower()
        if s not in ("speaking", "idle"):
            return

        was = self.tts_speaking
        self.tts_speaking = (s == "speaking")

        if self.tts_speaking and not was:
            # speaking blocca ascolto
            self.get_logger().info("TTS=speaking -> mode=speaking")
            self._cancel_resume_timer()
            # quando arriva speaking, il fallback è comunque armato: va bene, ma possiamo disarmarlo
            # e riarmarlo solo se serve su idle; però lasciarlo non rompe (timer one-shot).
            self.apply_mode("speaking")
            return

        if (not self.tts_speaking) and was:
            # idle -> ripristino ascolto (se in listen/demo)
            self.get_logger().info("TTS=idle -> ripristino")
            self._cancel_tts_fallback()
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
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
