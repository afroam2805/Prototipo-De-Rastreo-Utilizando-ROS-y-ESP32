from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('vision_udp')
    rviz_config = os.path.join(pkg_share, 'rviz', 'vision_udp.rviz')

    udp_node = Node(
        package='vision_udp',
        executable='udp_receiver',
        name='udp_receiver',
        output='screen',
        parameters=[
            {'listen_ip': '0.0.0.0'},
            {'port': 5005},
            {'frame_id': 'camera'}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([udp_node, rviz_node])