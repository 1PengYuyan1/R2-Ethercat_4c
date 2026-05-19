import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 手柄驱动
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]
        ),

        # 2. 解算节点
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            output='screen',
            parameters=[{'max_speed': 1.5, 'btn_enable': 4}]
        ),

        # 3. 串口通讯节点 
        Node(
            package='linkx_soem_demo',
            executable='stm32_node_cpp',
            name='stm32_node_chassis', # 名字必须唯一
            output='screen',
            parameters=[{
                'input_cmd_topic': '/cmd_vel',
                'input_buttons_topic': '/robot_buttons',
                'output_cmd_topic': '/chassis/cmd_vel',
                'output_buttons_topic': '/chassis/buttons'
            }]
        ),

        # 4. 串口桥接 
        Node(
            package='linkx_soem_demo',
            executable='stm32_node_cpp',
            name='stm32_node_gimbal', # 名字必须唯一
            output='screen',
            parameters=[{
                'input_cmd_topic': '/cmd_vel_dummy',
                'input_buttons_topic': '/robot_buttons',
                'output_cmd_topic': '/gimbal/cmd_vel',
                'output_buttons_topic': '/gimbal/buttons'
            }]
        )

    ])
