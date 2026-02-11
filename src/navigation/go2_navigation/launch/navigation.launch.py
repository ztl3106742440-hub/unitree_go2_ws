from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    完整的导航启动文件
    包含：
      1. Go2 驱动（里程计、TF）
      2. 点云转激光扫描
      3. 地图服务器和定位
      4. Nav2 导航栈

    使用方法：
      ros2 launch go2_navigation navigation.launch.py map:=/path/to/your/map.yaml
    """

    # 获取功能包路径
    go2_driver_pkg = get_package_share_directory('go2_driver_py')
    go2_sensors_pkg = get_package_share_directory('go2_sensors')
    go2_navigation_pkg = get_package_share_directory('go2_navigation')

    # 参数声明
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(go2_navigation_pkg, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file'
    )

    # 1. 启动 Go2 驱动
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

    # 3. 启动 Nav2 导航栈（包含定位和导航）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_navigation_pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        use_rviz,
        map_yaml_file,
        go2_driver_launch,
        pointcloud_to_scan_launch,
        nav2_launch,
    ])
