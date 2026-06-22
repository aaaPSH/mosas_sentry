#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pb_rm_simulation",
            executable="mecanum_keyboard.py",
            name="mecanum_keyboard",
            output="screen",
            emulate_tty=True,
        )
    ])
