import os
from launch import LaunchDescription
from launch.logging import launch_config
from launch_ros.actions import Node
import logging

def generate_launch_description():
    launch_config.level = logging.WARNING

    return LaunchDescription([
        # 1. 手柄驱动
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 100.0}]
        ),

        # 2. 解算节点
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[{
                'cmd_topic': '/chassis/remote_cmd_vel',
                'buttons_topic': '/robot_buttons'
            }]
        ),

        # 3. 底盘桥接
        Node(
            package='linkx_soem_demo',
            executable='stm32_node_cpp',
            name='stm32_node_chassis', # 名字必须唯一
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[{
                'input_cmd_topic': '/cmd_vel',
                'input_buttons_topic': '/robot_buttons',
                'output_cmd_topic': '/chassis/cmd_vel',
                'output_buttons_topic': '/chassis/buttons'
            }]
        ),

        # 4. 云台桥接
        Node(
            package='linkx_soem_demo',
            executable='stm32_node_cpp',
            name='stm32_node_gimbal', # 名字必须唯一
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[{
                'input_cmd_topic': '/cmd_vel_dummy',
                'input_buttons_topic': '/robot_buttons',
                'output_cmd_topic': '/gimbal/cmd_vel',
                'output_buttons_topic': '/gimbal/buttons'
            }]
        )

    ])
