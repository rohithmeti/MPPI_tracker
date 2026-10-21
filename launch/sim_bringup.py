import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # TurtleBot3 launch files need this; default to burger so no export is needed
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            [os.path.join(get_package_share_directory('nav'), 'config', 'sim.rviz')]
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Old tracker (replaced by MPPI below)
    # pure_pursuit_tracker = Node(
    #     package="nav",
    #     executable="pure_pursuit",
    #     name="pure_pursuit",
    #     parameters=[{
    #         'use_sim_time': True,
    #         'debug': True,
    #         'control_hz': 30.0,
    #         'lookahead_distance': 0.5,
    #         'lookahead_time': 1.5,
    #         'min_lookahead_distance': 0.3,
    #         'max_lookahead_distance': 0.9,
    #         'use_velocity_scaled_lookahead': True,
    #         'desired_linear_vel': 0.3,
    #         'desired_angular_vel': 0.35,
    #         'use_regulated_linear_velocity_scaling': True,
    #         'regulated_linear_scaling_min_radius': 0.90,
    #         'velocity_scaling_distance': 0.5,
    #         'v_max': 0.5,
    #         'dv_max': 0.1,
    #         'w_max': 0.5,
    #         'dw_max': 0.25,
    #         'k_rot': 1.0,
    #         'goal_position_tolerance': 0.2,
    #         'use_rotate_to_heading': True,
    #         'rotate_to_heading_min_angle': 0.785,
    #         'point_threshold': 15,
    #         'obstacle_avoidance_distance': 1.0,
    #     }]
    # )

    # Waypoints (CSV) -> smooth path on /path
    path_smoother = Node(
        package='nav',
        executable='path_smoother.py',
        name='path_smoother',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # /path -> MPPI, one waypoint at a time
    path_to_mppi = Node(
        package='nav',
        executable='path_to_mppi.py',
        name='path_to_mppi',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Nav2 controller server running MPPI (settings in config/mppi.yaml)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[os.path.join(get_package_share_directory('nav'), 'config', 'mppi.yaml')]
    )

    # Starts (configures and activates) the controller server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server']
        }]
    )

    nodes = []
    nodes.append(turtlebot3_gazebo)
    # nodes.append(pure_pursuit_tracker)
    nodes.append(controller_server)
    nodes.append(lifecycle_manager)
    nodes.append(path_smoother)
    nodes.append(path_to_mppi)
    nodes.append(rviz)

    return LaunchDescription(nodes)
