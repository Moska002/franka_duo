from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

import os
import xacro
from ament_index_python.packages import get_package_share_directory

# LIBGL_ALWAYS_SOFTWARE=1

def generate_single_robot(robot_name, x, y, z):
    xacro_file = 'fr3.urdf.xacro'

    xacro_complete_path =  os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        xacro_file
    )
    urdf = xacro.process_file(xacro_complete_path, mappings={
        # 'ros2_control': 'true',
        'gazebo': 'true'
    })

    robot_description = urdf.toxml()

    robot_description = robot_description.replace(
        '<robot name="fr3">',
        '<robot name="' + robot_name + '">'
    )

    return GroupAction([
        PushRosNamespace(robot_name),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['joint_states'],
                'rate': 30
            }],
        ), 

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-entity', robot_name,
                '-topic', f"/{robot_name}/robot_description",
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ],
            output='screen',
        ),
    ])


def generate_launch_description():
    left_namespace = DeclareLaunchArgument('left_namespace', default_value='left')
    right_namespace = DeclareLaunchArgument('right_namespace', default_value='right')

    rviz_config_file = 'fr3.rviz'

    rviz_path = os.path.join(get_package_share_directory('franka_duo'), 'rviz', rviz_config_file)

    # Gazebo specific configurations
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    #launch.extend(generate_single_robot(LaunchConfiguration('left_namespace')))
    #launch.extend(generate_single_robot(LaunchConfiguration('right_namespace')))

    launch = [
        left_namespace,
        right_namespace
    ]
    launch.extend([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': 'empty.sdf -r --render-engine ogre'}.items(),
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        ),

        generate_single_robot('left', 0, 1, 0),
        generate_single_robot('right', 0, -1, 0)
    ])

    return LaunchDescription(launch)

#Node(
#    package='ros_gz_sim',
#    executable='create',
#    namespace='right',
#    arguments=[
#        '-entity', 'right',
#        '-topic', '/right' + '/robot_description',
#        '-robot_namespace', 'right',
#        '-x', '5',
#        '-y', '0',
#        '-z', '0'
#    ],
#    output='screen',
#)