from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    将 Unitree Go2 的 UTlidar 点云转换为 2D 激光扫描
    用于 SLAM 和导航
    """

    pkg_share = get_package_share_directory('go2_sensors')
    params_file = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan_params.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[params_file],
            remappings=[
                ('cloud_in', '/utlidar/cloud_deskewed'),
                ('scan', '/scan')
            ],
            output='screen'
        )
    ])
