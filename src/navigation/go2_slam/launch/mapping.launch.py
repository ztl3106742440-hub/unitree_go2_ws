from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    完整的建图启动文件
    包含：
      1. Go2 驱动（里程计、TF、关节状态）
      2. 点云转激光扫描
      3. SLAM Toolbox 建图
    """

    # 获取功能包路径
    go2_driver_pkg = get_package_share_directory('go2_driver_py')
    go2_sensors_pkg = get_package_share_directory('go2_sensors')
    go2_slam_pkg = get_package_share_directory('go2_slam')

    # 参数声明
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # 1. 启动 Go2 驱动（提供 odom、TF、关节状态等）
    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items()
    )

    # 2. 启动点云转激光扫描
    pointcloud_to_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_sensors_pkg, 'launch', 'pointcloud_to_laserscan.launch.py')
        )
    )

    # 3. 启动 SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_slam_pkg, 'launch', 'slam_toolbox.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false'  # 驱动已经启动了 RViz
        }.items()
    )

    return LaunchDescription([
        use_rviz,
        go2_driver_launch,
        pointcloud_to_scan_launch,
        slam_launch,
    ])
