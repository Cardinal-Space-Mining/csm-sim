import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    sim_pkg_path = get_package_share_directory('csm_sim')

    # launch isaac sim
    isaac_sim = ExecuteProcess(
        cmd = [
            PythonExpression(["'", LaunchConfiguration('isaac-root'), "' + 'python.sh'" ]),
            os.path.join(sim_pkg_path, 'launch', 'sim.py'),
            '--gui', 'true',
            '--assets', os.path.join(sim_pkg_path, 'isaac-assets/')
        ],
        output = 'screen',
        # prefix = ['xterm -e']
    )
    # launch xbox control
    launch_xbox_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'xbox_ctrl.launch.py')
        )
    )
    # robot state publisher
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = { 'use_sim_time' : 'true' }.items()
    )
    # foxglove
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('isaac-root', default_value=os.path.join(os.getenv('HOME'), '.local/share/ov/pkg/isaac-sim-4.2.0/')),
        DeclareLaunchArgument('foxglove', default_value='true'),
        isaac_sim,
        launch_xbox_ctrl,
        state_publisher,
        foxglove_node,
    ])
