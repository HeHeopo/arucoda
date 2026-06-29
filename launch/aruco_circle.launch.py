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
        DeclareLaunchArgument('spin_angular_speed', default_value='0.4'),
        DeclareLaunchArgument('total_markers', default_value='4'),
        DeclareLaunchArgument('goal_frame', default_value='map'),
        DeclareLaunchArgument('parent_frame', default_value='camera_link'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_delay', default_value='3.0'),

        Node(
            package='arucoda',
            executable='aruco_circle',
            name='aruco_circle',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'marker_size': LaunchConfiguration('marker_size'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'spin_angular_speed': LaunchConfiguration('spin_angular_speed'),
                'total_markers': LaunchConfiguration('total_markers'),
                'goal_frame': LaunchConfiguration('goal_frame'),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
                'initial_pose_delay': LaunchConfiguration('initial_pose_delay'),
            }],
        ),
    ])
