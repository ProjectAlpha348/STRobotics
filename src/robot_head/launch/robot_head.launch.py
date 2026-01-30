from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- Launch args (Vision) ----------------
    camera_index = LaunchConfiguration("camera_index")
    camera_device = LaunchConfiguration("camera_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    conf_thres = LaunchConfiguration("conf_thres")
    iou_thres = LaunchConfiguration("iou_thres")
    target_class = LaunchConfiguration("target_class")
    publish_annotated = LaunchConfiguration("publish_annotated")
    rate_hz = LaunchConfiguration("rate_hz")
    model_relpath = LaunchConfiguration("model_relpath")

    # ---------------- Launch args (Voice) ----------------
    audio_device = LaunchConfiguration("audio_device")
    input_sample_rate = LaunchConfiguration("input_sample_rate")
    vosk_sample_rate = LaunchConfiguration("vosk_sample_rate")
    auto_start = LaunchConfiguration("auto_start")
    enable_partial = LaunchConfiguration("enable_partial")
    min_text_len = LaunchConfiguration("min_text_len")
    resample_to_vosk_rate = LaunchConfiguration("resample_to_vosk_rate")
    voice_text_topic = LaunchConfiguration("voice_text_topic")

    # ---------------- Launch args (Orchestrator/Eyes) ----------------
    eyes_cmd_topic = LaunchConfiguration("eyes_cmd_topic")
    mode_topic = LaunchConfiguration("mode_topic")
    listen_resume_delay_ms = LaunchConfiguration("listen_resume_delay_ms")
    default_state = LaunchConfiguration("default_state")

    # Speak/Voice topics (allineati al tuo speak_node)
    speech_say_topic = LaunchConfiguration("speech_say_topic")
    tts_state_topic = LaunchConfiguration("tts_state_topic")
    voice_cmd_topic = LaunchConfiguration("voice_cmd_topic")
    vision_enable_topic = LaunchConfiguration("vision_enable_topic")

    # ---------------- Launch args (Sensors) ----------------
    env_topic = LaunchConfiguration("env_topic")
    i2c_address = LaunchConfiguration("i2c_address")
    env_update_hz = LaunchConfiguration("env_update_hz")
    env_stale_timeout_s = LaunchConfiguration("env_stale_timeout_s")

    # Orchestrator fallback (se lo stato TTS è intermittente)
    tts_fallback_resume_s = LaunchConfiguration("tts_fallback_resume_s")

    # ---------------- Launch args (Eyes hardware) ----------------
    i2c_bus = LaunchConfiguration("i2c_bus")
    tca_addr = LaunchConfiguration("tca_addr")
    oled_addr = LaunchConfiguration("oled_addr")
    left_channel = LaunchConfiguration("left_channel")
    right_channel = LaunchConfiguration("right_channel")
    rotate = LaunchConfiguration("rotate")

    # Blink defaults
    blink_enabled = LaunchConfiguration("blink_enabled")
    blink_min_s = LaunchConfiguration("blink_min_s")
    blink_max_s = LaunchConfiguration("blink_max_s")

    # ---------------- Nodes ----------------
    vision_node = Node(
        package="robot_head",
        executable="vision_node",
        name="vision",
        output="screen",
        parameters=[{
            "camera_index": camera_index,
            "camera_device": camera_device,
            "image_width": image_width,
            "image_height": image_height,
            "conf_thres": conf_thres,
            "iou_thres": iou_thres,
            "target_class": target_class,
            "publish_annotated": publish_annotated,
            "rate_hz": rate_hz,
            "model_relpath": model_relpath,
        }],
    )

    voice_node = Node(
        package="robot_head",
        executable="voice_node",
        name="voice",
        output="screen",
        parameters=[{
            "device": audio_device,
            "input_sample_rate": input_sample_rate,
            "vosk_sample_rate": vosk_sample_rate,
            "auto_start": auto_start,
            "enable_partial": enable_partial,
            "min_text_len": min_text_len,
            "resample_to_vosk_rate": resample_to_vosk_rate,

            # (facoltativo) se vuoi usare questo topic anche dentro voice_node in futuro
            # "text_topic": voice_text_topic,
        }],
    )

    # NEW: sensore ambiente (BME280) -> pubblica /tommy/sensors/env
    environment_node = Node(
        package="robot_head",
        executable="environment_node",
        name="environment",
        output="screen",
        parameters=[{
            "i2c_address": i2c_address,
            "update_hz": env_update_hz,
            "env_topic": env_topic,
        }],
    )

    speak_node = Node(
        package="robot_head",
        executable="speak_node",
        name="speak",
        output="screen",
        # parameters=[{"say_topic": speech_say_topic, "state_topic": tts_state_topic}],
    )

    eyes_node = Node(
        package="robot_head",
        executable="eyes_node",
        name="eyes",
        output="screen",
        parameters=[{
            "cmd_topic": eyes_cmd_topic,
            "i2c_bus": i2c_bus,
            "tca_addr": tca_addr,
            "oled_addr": oled_addr,
            "left_channel": left_channel,
            "right_channel": right_channel,
            "rotate": rotate,
            "blink_enabled": blink_enabled,
            "blink_min_s": blink_min_s,
            "blink_max_s": blink_max_s,
        }],
    )

    orchestrator_node = Node(
        package="robot_head",
        executable="head_orchestrator",
        name="orchestrator",
        output="screen",
        parameters=[{
            "eyes_cmd_topic": eyes_cmd_topic,
            "mode_topic": mode_topic,
            "listen_resume_delay_ms": listen_resume_delay_ms,
            "default_state": default_state,

            "speech_say_topic": speech_say_topic,
            "tts_state_topic": tts_state_topic,
            "voice_cmd_topic": voice_cmd_topic,
            "vision_enable_topic": vision_enable_topic,

            # allineamento: il tuo orchestrator usa /tommy/voice/text
            "voice_text_topic": voice_text_topic,

            # sensori
            "env_topic": env_topic,
            "env_stale_timeout_s": env_stale_timeout_s,

            # fallback ripartenza ascolto
            "tts_fallback_resume_s": tts_fallback_resume_s,
        }],
    )

    # ---------------- If orchestrator exits => shutdown whole launch ----------------
    shutdown_on_orchestrator_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=orchestrator_node,
            on_exit=[Shutdown(reason="Orchestrator exited -> shutdown all nodes")],
        )
    )

    # ---------------- LaunchDescription ----------------
    return LaunchDescription([
        # Vision args
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("camera_device", default_value=""),  # es: /dev/video0
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("conf_thres", default_value="0.35"),
        DeclareLaunchArgument("iou_thres", default_value="0.45"),
        DeclareLaunchArgument("target_class", default_value="-1"),  # -1=tutte, 0=person
        DeclareLaunchArgument("publish_annotated", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("model_relpath", default_value="models/yolo/yolov8n.pt"),

        # Voice args
        DeclareLaunchArgument("audio_device", default_value="-1"),
        DeclareLaunchArgument("input_sample_rate", default_value="48000"),
        DeclareLaunchArgument("vosk_sample_rate", default_value="16000"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("enable_partial", default_value="false"),
        DeclareLaunchArgument("min_text_len", default_value="2"),
        DeclareLaunchArgument("resample_to_vosk_rate", default_value="true"),
        DeclareLaunchArgument("voice_text_topic", default_value="/tommy/voice/text"),

        # Orchestrator/Eyes topics
        DeclareLaunchArgument("eyes_cmd_topic", default_value="/robot_head/eyes/cmd"),
        DeclareLaunchArgument("mode_topic", default_value="/robot_head/state"),
        DeclareLaunchArgument("listen_resume_delay_ms", default_value="250"),
        DeclareLaunchArgument("default_state", default_value="idle"),

        # Speak/Voice/Vision topics
        DeclareLaunchArgument("speech_say_topic", default_value="/tommy/speech/say"),

        # IMPORTANT: qui tieni il topic che davvero pubblica speaking/idle
        # Nel tuo sistema dai log sembra funzionare. Se serve, lo cambi qui.
        DeclareLaunchArgument("tts_state_topic", default_value="/robot/tts/state"),

        DeclareLaunchArgument("voice_cmd_topic", default_value="/tommy/voice/cmd"),
        DeclareLaunchArgument("vision_enable_topic", default_value="/tommy/vision/enable"),

        # Sensors args
        DeclareLaunchArgument("env_topic", default_value="/tommy/sensors/env"),
        DeclareLaunchArgument("i2c_address", default_value="118"),  # 0x76 = 118
        DeclareLaunchArgument("env_update_hz", default_value="2.0"),
        DeclareLaunchArgument("env_stale_timeout_s", default_value="30.0"),

        # Orchestrator fallback
        DeclareLaunchArgument("tts_fallback_resume_s", default_value="20.0"),

        # Eyes hardware
        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("tca_addr", default_value="112"),  # 0x70 = 112
        DeclareLaunchArgument("oled_addr", default_value="60"),  # 0x3C = 60
        DeclareLaunchArgument("left_channel", default_value="0"),
        DeclareLaunchArgument("right_channel", default_value="1"),
        DeclareLaunchArgument("rotate", default_value="0"),

        # Blink tuning
        DeclareLaunchArgument("blink_enabled", default_value="true"),
        DeclareLaunchArgument("blink_min_s", default_value="3.0"),
        DeclareLaunchArgument("blink_max_s", default_value="7.0"),

        # Nodes (ordine: sensore prima dell’orchestrator)
        vision_node,
        voice_node,
        environment_node,
        speak_node,
        eyes_node,
        orchestrator_node,

        # Shutdown behavior
        shutdown_on_orchestrator_exit,
    ])
