from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "topic",
            description="a pointcloud topic to process",
            default_value="/lidar_points" 
        ),

        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("topic")},

                # ====================================================
                # 1. 根据 cloud_bounds.py 自动计算的地图中心点
                # ====================================================
                {'position_x': 43.04787826538086},  
                {'position_y': -12.594022750854492},    

                # ====================================================
                # 2. 地图画布的长宽 
                # ====================================================
                {'length_x': 2000.0},
                {'length_y': 1450.0},
                {'cell_size': 0.2},

                # ====================================================
                # 3. Z轴（高度）过滤
                {'obstacle_min_z': -0.4},
                {'obstacle_max_z': 3.0},

                # 输出坐标系改为脚本里识别到的
                {'frame_out': 'hesai_lidar'}, 

                # 话题名称
                {'mapi_topic_name': 'intensity_grid'},
                {'maph_topic_name': 'height_grid'},
                {'acc_mapi_topic_name': 'acc_intensity_grid'},
                {'acc_maph_topic_name': 'acc_height_grid'},
            ]
        )
    ])
