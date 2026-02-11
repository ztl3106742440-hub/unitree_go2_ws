from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    使用已有地图进行定位（AMCL）
    用于：在已知地图中定位机器人位置
    """

    # 获取功能包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    go2_navigation_dir = get_package_share_directory('go2_navigation')

    # 参数声明
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(go2_navigation_dir, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(go2_navigation_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    # 启动 Nav2 Localization
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        map_yaml_file,
        params_file,
        nav2_localization_launch,
    ])
