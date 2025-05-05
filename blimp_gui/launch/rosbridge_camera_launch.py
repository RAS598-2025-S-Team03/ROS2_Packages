from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),
        Node(
            package='blimp_gui',
            executable='webserver',
            name='web_gui_server',
            output='screen'
        )
        #ExecuteProcess(
        #    cmd=['python3', '/home/nihar/blimp_ws/install/blimp_gui/lib/blimp_gui/export_graph.py'],
        #    shell=False,
        #    output='screen'
        #)
    ])

