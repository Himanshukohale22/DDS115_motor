import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = get_package_share_directory('fynd_bot_control')
    urdf_pkg_name = get_package_share_directory('plan_bot_description')
    urdf_path = os.path.join(urdf_pkg_name,'description','plan_bot.urdf')
    rviz_path = os.path.join(urdf_pkg_name,'rviz','plan_bot.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_path]
    )

    left_wheel_joint_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_wheel_join',
        output='screen',
        arguments=[
            '-0.15','0.24','-0.0225',
            '0','0','1.5708',
            'base_link',
            'left_wheel'
        ]
    )

    right_wheel_joint_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_wheel_join',
        output='screen',
        arguments=[
            '-0.15','-0.24','-0.0225',
            '0','0','1.5708',
            'base_link',
            'right_wheel'
        ]
    )

    odom_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_frame',
        output='screen',
        arguments=[
            '0','o','0',
            '0','0','0','1',
            'odom',
            'base_link'
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        left_wheel_joint_node,
        right_wheel_joint_node

    ])
