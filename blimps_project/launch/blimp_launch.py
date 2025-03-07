import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['sudo', 'pigpiod'],
            output='screen'
        ),
	    Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='blimps_project',
            executable='camera',
            name='webcam_publisher',
            output='screen'
        ),
        Node(
            package='blimps_project',
            executable='blob',
            name='blob_detector',
            output='screen'
        ),
         Node(
            package='blimps_project',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
        Node(
            package='blimps_project',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ),
    ])

