from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ifname = LaunchConfiguration('ifname')
    max_speed = LaunchConfiguration('max_speed')
    start_vehicle_control = LaunchConfiguration('start_vehicle_control')
    start_gimbal_bridge = LaunchConfiguration('start_gimbal_bridge')
    vehicle_prefix = LaunchConfiguration('vehicle_prefix')
    ros_nodes_prefix = LaunchConfiguration('ros_nodes_prefix')

    return LaunchDescription([
        DeclareLaunchArgument('ifname', default_value='enxf01e341224fd'),
        DeclareLaunchArgument('max_speed', default_value='1.5'),
        DeclareLaunchArgument('start_vehicle_control', default_value='true'),
        DeclareLaunchArgument('start_gimbal_bridge', default_value='false'),
        DeclareLaunchArgument('vehicle_prefix', default_value=''),
        DeclareLaunchArgument('ros_nodes_prefix', default_value=''),

        # 1) 手柄驱动
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            prefix=ros_nodes_prefix,
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
            output='screen',
        ),

        # 2) 遥控解算节点：/joy → /cmd_vel + /robot_buttons
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            prefix=ros_nodes_prefix,
            output='screen',
            parameters=[{'max_speed': max_speed}],
        ),

        # 3) 话题转发（底盘）：/cmd_vel → /chassis/cmd_vel；/robot_buttons → /chassis/buttons
        #    其它传感器节点可以直接发布 /cmd_vel（与手柄共用入口）
        #    或绕过手柄直接发布 /chassis/cmd_vel
        Node(
            package='linkx_soem_demo',
            executable='stm32_node_cpp',
            name='chassis_relay',
            prefix=ros_nodes_prefix,
            output='screen',
            parameters=[{
                'input_cmd_topic': '/cmd_vel',
                'input_buttons_topic': '/robot_buttons',
                'output_cmd_topic': '/chassis/cmd_vel',
                'output_buttons_topic': '/chassis/buttons',
            }],
        ),

        # 4) 话题转发（云台，可选）
        Node(
            package='linkx_soem_demo',
            executable='stm32_node_cpp',
            name='gimbal_relay',
            prefix=ros_nodes_prefix,
            output='screen',
            condition=IfCondition(start_gimbal_bridge),
            parameters=[{
                'input_cmd_topic': '/cmd_vel_dummy',
                'input_buttons_topic': '/robot_buttons',
                'output_cmd_topic': '/gimbal/cmd_vel',
                'output_buttons_topic': '/gimbal/buttons',
            }],
        ),

        # 5) 整车主控（SOEM/EtherCAT 主站 + LinkX-4C CAN 桥 + r2 chassis/gantry/arm/navigation）
        #    需要 raw 以太网权限：vehicle_prefix='sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH'
        #    或预先 setcap cap_net_raw,cap_net_admin+ep <linkx_soem_demo>
        Node(
            package='linkx_soem_demo',
            executable='linkx_soem_demo',
            name='vehicle_control',
            output='screen',
            condition=IfCondition(start_vehicle_control),
            prefix=vehicle_prefix,
            arguments=[ifname],
        ),
    ])
