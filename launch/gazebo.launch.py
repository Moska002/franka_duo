import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    xacro_file = 'fr3.urdf.xacro'
    rviz_config_file = 'fr3.rviz'
    
    xacro_complete_path =  os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        xacro_file
    )
    urdf = xacro.process_file(xacro_complete_path,  mappings={
        'ros2_control': 'true',
        'gazebo': 'true'
    })
    robot_description = urdf.toprettyxml(indent='\t')

    print(robot_description)

    rviz_path = os.path.join(get_package_share_directory('franka_duo'), 'rviz', rviz_config_file)

    # Gazebo specific configurations
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    joint_velocity_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_position_example_controller'],
        output='screen'
    )


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': 'empty.sdf -r --render-engine ogre'}.items(),
        ),
        
        spawn,

        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[joint_velocity_example_controller],
            )
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['joint_states'],
                'rate': 30
            }],
        )
    ])