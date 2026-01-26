from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch arguments ----
    use_vision = LaunchConfiguration('use_vision')
    use_voice  = LaunchConfiguration('use_voice')
    use_speak  = LaunchConfiguration('use_speak')

    declare_use_vision = DeclareLaunchArgument(
        'use_vision',
        default_value='true',
        description='Avvia il nodo di visione'
    )

    declare_use_voice = DeclareLaunchArgument(
        'use_voice',
        default_value='true',
        description='Avvia il nodo di riconoscimento vocale'
    )

    declare_use_speak = DeclareLaunchArgument(
        'use_speak',
        default_value='true',
        description='Avvia il nodo text-to-speech'
    )

    # ---- Nodes ----
    vision_node = Node(
        package='robot_head',
        executable='vision_node',
        name='vision_node',
        output='screen',
        condition=None
    )

    voice_node = Node(
        package='robot_head',
        executable='voice_node',
        name='voice_node',
        output='screen',
        condition=None
    )

    speak_node = Node(
        package='robot_head',
        executable='speak_node',
        name='speak_node',
        output='screen',
        condition=None
    )

    orchestrator_node = Node(
        package='robot_head',
        executable='orchestrator_node',
        name='orchestrator',
        output='screen'
    )

    # ---- Launch description ----
    return LaunchDescription([
        declare_use_vision,
        declare_use_voice,
        declare_use_speak,

        orchestrator_node,
        vision_node,
        voice_node,
        speak_node,
    ])
