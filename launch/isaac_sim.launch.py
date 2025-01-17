import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

ISAAC_SIM_DEFAULT_VERSION = '4.2.0'
ISAAC_SIM_DEFAULT_INSTALL_PATH = f'.local/share/ov/pkg/isaac-sim-{ISAAC_SIM_DEFAULT_VERSION}/'


def generate_launch_description():

    pkg_path = get_package_share_directory('csm_sim')

# CORE
    # launch isaac sim
    isaac_sim = ExecuteProcess(
        cmd = [
            PythonExpression(["'", LaunchConfiguration('isaac-root'), "' + 'python.sh'" ]),
            os.path.join(pkg_path, 'scripts', 'isaac_sim.py'),
            '--gui', 'true',
            '--assets', os.path.join(pkg_path, 'isaac-assets/')
        ],
        output = 'screen',
        # prefix = ['xterm -e']
    )

# OPTIONAL
    # launch xbox control
    launch_xbox_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'xbox_ctrl.launch.py')
        ),
        condition = IfCondition(LaunchConfiguration('xbox_ctrl', default='true'))
    )
    # robot state publisher
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = { 'use_sim_time' : 'true' }.items(),
        condition = IfCondition(LaunchConfiguration('state_pub', default='true'))
    )
    # foxglove
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = { 'use_sim_time' : 'true' }.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('isaac-root', default_value=os.path.join(os.getenv('HOME'), ISAAC_SIM_DEFAULT_INSTALL_PATH)),
        DeclareLaunchArgument('xbox_ctrl', default_value='true'),
        DeclareLaunchArgument('state_pub', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        isaac_sim,
        launch_xbox_ctrl,
        state_publisher,
        foxglove_node,
    ])
