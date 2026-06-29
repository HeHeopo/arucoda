from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_size', default_value='0.10'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),

        Node(
            package='arucoda',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[{
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'marker_size': LaunchConfiguration('marker_size'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
            }],
        ),
    ])
