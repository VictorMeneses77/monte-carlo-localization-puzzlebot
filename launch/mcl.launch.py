import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_localization')
    map_yaml = os.path.join(pkg_share, 'maps', 'obstacles.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'mcl.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Abrir RViz para visualizar partículas y pose estimada'
    )

    mcl_node = Node(
        package='puzzlebot_localization',
        executable='monte_carlo_localization',
        name='monte_carlo_localization',
        output='screen',
        parameters=[{
            'map_yaml_path': map_yaml,
            'num_particles': 300,
            'motion_noise_xy': 0.05,
            'motion_noise_theta': 0.05,
            'scan_subsample': 20,
            'resample_rate': 0.5,
            'initial_pose': [0.0, 0.0, 0.0],
            'initial_pose_std': [0.3, 0.3, 0.2],
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        mcl_node,
        rviz_node,
    ])
