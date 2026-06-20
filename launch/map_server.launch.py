from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'frame_id': 'world'},
        ],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/zylyehuo/mapping/pointcloud_to_grid/src/pointcloud_to_grid/MAP/truck/accumulated_bw_map.yaml'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        map_server_node,
        lifecycle_manager_node,
    ])

