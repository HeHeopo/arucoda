from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('frame_width', default_value='1280'),
        DeclareLaunchArgument('frame_height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='24'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_size', default_value='0.03'),
        DeclareLaunchArgument('camera_frame', default_value='camera_frame'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('fx', default_value='836.51438028'),
        DeclareLaunchArgument('fy', default_value='835.73816422'),
        DeclareLaunchArgument('k1', default_value='-0.25977023'),
        DeclareLaunchArgument('k2', default_value='0.17973683'),
        DeclareLaunchArgument('p1', default_value='-0.00320421'),
        DeclareLaunchArgument('p2', default_value='-0.00957875'),
        DeclareLaunchArgument('k3', default_value='-0.086389'),

        Node(
            package='arucoda',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'marker_size': LaunchConfiguration('marker_size'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'fx': LaunchConfiguration('fx'),
                'fy': LaunchConfiguration('fy'),
                'k1': LaunchConfiguration('k1'),
                'k2': LaunchConfiguration('k2'),
                'p1': LaunchConfiguration('p1'),
                'p2': LaunchConfiguration('p2'),
                'k3': LaunchConfiguration('k3'),
            }],
        ),
    ])
