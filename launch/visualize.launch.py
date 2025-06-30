from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_file = 'fr3.urdf.xacro'
    rviz_config_file = 'fr3.rviz'
    
    xacro_file =  os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        xacro_file
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    rviz_path = os.path.join(get_package_share_directory('franka_duo'), 'rviz', rviz_config_file)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )
    ])