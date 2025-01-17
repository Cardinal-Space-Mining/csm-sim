import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('csm_sim')

    launch_file_dir = os.path.join( pkg_path, 'launch' )

    # simulator core
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( launch_file_dir, 'gazebo_sim.launch.py' )
        ),
        launch_arguments = {
            'gz_gui' : LaunchConfiguration('gz_gui', default = 'true')
        }.items()
    )
    # robot state publisher (ROS tf tree)
    launch_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( launch_file_dir, 'robot_state_publisher.launch.py' )
        ),
        launch_arguments = {
            'use_sim_time' : 'true'
        }.items()
    )
    # joy and teleop nodes for xbox
    launch_xbox_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( launch_file_dir, 'xbox_ctrl.launch.py' )
        )
    )
    # rviz
    launch_rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', os.path.join( pkg_path, 'config', 'sim_demo.rviz' )],
        condition = IfCondition( LaunchConfiguration('rviz', default = 'false') ),
        parameters = [{'use_sim_time' : True}]
    )
    # foxglove
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value = 'true'),
        DeclareLaunchArgument('rivz', default_value = 'false'),
        DeclareLaunchArgument('foxglove', default_value = 'true'),
        launch_gazebo,
        launch_state_pub,
        launch_xbox_ctrl,
        launch_rviz,
        foxglove_node
    ])
