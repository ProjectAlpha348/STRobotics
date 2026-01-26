from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- Launch args ----------------
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

    audio_device = LaunchConfiguration("audio_device")
    input_sample_rate = LaunchConfiguration("input_sample_rate")
    vosk_sample_rate = LaunchConfiguration("vosk_sample_rate")
    auto_start = LaunchConfiguration("auto_start")
    enable_partial = LaunchConfiguration("enable_partial")
    min_text_len = LaunchConfiguration("min_text_len")
    resample_to_vosk_rate = LaunchConfiguration("resample_to_vosk_rate")

    # ---- Coordinamento orchestrator (topic standard) ----
    topic_state = LaunchConfiguration("topic_state")
    topic_eyes_cmd = LaunchConfiguration("topic_eyes_cmd")
    topic_voice_cmd = LaunchConfiguration("topic_voice_cmd")
    topic_speak_cmd = LaunchConfiguration("topic_speak_cmd")
    topic_speak_event = LaunchConfiguration("topic_speak_event")

    return LaunchDescription([
        # ---------------- Arguments ----------------
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("camera_device", default_value=""),   # es: /dev/video0
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("conf_thres", default_value="0.35"),
        DeclareLaunchArgument("iou_thres", default_value="0.45"),
        DeclareLaunchArgument("target_class", default_value="-1"),  # -1=tutte, 0=person
        DeclareLaunchArgument("publish_annotated", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("model_relpath", default_value="models/yolo/yolov8n.pt"),

        DeclareLaunchArgument("audio_device", default_value="-1"),          # sounddevice index
        DeclareLaunchArgument("input_sample_rate", default_value="48000"),  # 44100/48000
        DeclareLaunchArgument("vosk_sample_rate", default_value="16000"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("enable_partial", default_value="false"),
        DeclareLaunchArgument("min_text_len", default_value="2"),
        DeclareLaunchArgument("resample_to_vosk_rate", default_value="true"),

        # ---- Topic standard (modifica qui se vuoi cambiare namespace) ----
        DeclareLaunchArgument("topic_state", default_value="/robot_head/state"),
        DeclareLaunchArgument("topic_eyes_cmd", default_value="/robot_head/eyes/cmd"),
        DeclareLaunchArgument("topic_voice_cmd", default_value="/robot_head/voice/cmd"),
        DeclareLaunchArgument("topic_speak_cmd", default_value="/robot_head/speak/cmd"),
        DeclareLaunchArgument("topic_speak_event", default_value="/robot_head/speak/event"),

        # ---------------- Nodes ----------------
        Node(
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
        ),

        Node(
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

                # Coordinamento (da implementare nel voice_node):
                "cmd_topic": topic_voice_cmd,   # ascolta comandi: listen=0/1, reset=1
            }],
        ),

        Node(
            package="robot_head",
            executable="speak_node",
            name="speak",
            output="screen",
            parameters=[{
                # Coordinamento (da implementare nel speak_node):
                "cmd_topic": topic_speak_cmd,       # riceve testo / cmd speak
                "event_topic": topic_speak_event,   # pubblica speaking=1/0
            }],
        ),

        Node(
            package="robot_head",
            executable="eyes_node",
            name="eyes",
            output="screen",
            parameters=[{
                # Coordinamento (da implementare nel eyes_node):
                "cmd_topic": topic_eyes_cmd,   # riceve: state=..., blink=0/1
            }],
        ),

        Node(
            package="robot_head",
            executable="head_orchestrator",
            name="orchestrator",
            output="screen",
            parameters=[{
                # Topic standard usati dall’orchestrator
                "topic_state": topic_state,
                "topic_eyes_cmd": topic_eyes_cmd,
                "topic_voice_cmd": topic_voice_cmd,
                "topic_speak_cmd": topic_speak_cmd,
                "topic_speak_event": topic_speak_event,

                # Policy base (puoi rifinirle)
                "speaking_disable_listen": True,
                "speaking_eyes_state": "attentive",
                "speaking_blink": 0,
                "listening_eyes_state": "alert",
                "listening_blink": 1,
                "idle_eyes_state": "normal",
                "idle_blink": 1,

                # anti-rientro audio: delay prima di riattivare listen
                "listen_resume_delay_ms": 250,
            }],
        ),
    ])
