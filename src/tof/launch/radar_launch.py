"""
上位机雷达数据解析与发布节点启动文件

用法:
  ros2 launch tof radar_launch.py                     # 使用默认网卡 enp86s0
  ros2 launch tof radar_launch.py ifname:=enp88s0     # 指定网卡
  sudo setcap cap_net_raw,cap_net_admin=eip /home/rc/R2/Ethercat-R2/install/tof/lib/tof/can_link_test
  ros2 launch tof radar_launch.py ifname:=enp3s0

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==========================================
    # 声明启动参数
    # ==========================================
    ifname_arg = DeclareLaunchArgument(
        'ifname',
        default_value='enp86s0',
        description='EtherCAT 绑定的网卡名 (e.g. enp86s0, eth0)'
    )

    baud_arg = DeclareLaunchArgument(
        'can_baud',
        default_value='1M',
        description='CAN 波特率: 1M, 500k, 250k'
    )

    slave_high_arg = DeclareLaunchArgument(
        'slave_high',
        default_value='1',
        description='高频传感器 DM-LinKX-4c EtherCAT 从站 ID'
    )

    slave_low_arg = DeclareLaunchArgument(
        'slave_low',
        default_value='0',
        description='低频传感器 DM-LinKX-4c EtherCAT 从站 ID，0 表示不启用'
    )

    start_up_range_subscriber_arg = DeclareLaunchArgument(
        'start_up_range_subscriber',
        default_value='true',
        description='是否启动四路 high up/down Range 订阅节点'
    )

    # ==========================================
    # 统一传感器节点（高频 + 低频）
    #   高频 TFmini-S (8路, slave 1):
    #     /high/front /high/left /high/right /high/back        (各加 /range)
    #     /high/up_front /high/up_back /high/down_front /high/down_back
    #   低频传感器 (4路, slave 2):
    #     /low/front /low/left /low/right /low/back            (各加 /range)
    # ==========================================
    radar_node = Node(
        package='tof',
        executable='can_link_test',
        name='upper_computer_radar_node',
        output='screen',
        arguments=[LaunchConfiguration('ifname'),
                   LaunchConfiguration('can_baud'),
                   LaunchConfiguration('slave_high'),
                   LaunchConfiguration('slave_low')],
        emulate_tty=True,
    )

    up_range_subscriber_node = Node(
        package='tof',
        executable='up_range_subscriber',
        name='up_range_subscriber',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_up_range_subscriber')),
        parameters=[{
            'topic_up_front': '/high/up_front/range',
            'topic_up_back': '/high/up_back/range',
            'topic_down_front': '/high/down_front/range',
            'topic_down_back': '/high/down_back/range',
            'log_rate_hz': 5.0,
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        ifname_arg,
        baud_arg,
        slave_high_arg,
        slave_low_arg,
        start_up_range_subscriber_arg,
        radar_node,
        up_range_subscriber_node,
    ])
