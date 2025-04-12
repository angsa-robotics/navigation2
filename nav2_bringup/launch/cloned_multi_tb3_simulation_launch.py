# Copyright (c) 2023 LG Electronics.
# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import ParseMultiRobotPose


def count_robots(context: LaunchContext) -> list[LogInfo]:
    """Count the number of robots from the 'robots' launch argument."""
    robots_str = LaunchConfiguration('robots').perform(context).strip()
    log_msg = ''

    if not robots_str:
        log_msg = 'No robots provided in the launch argument.'

    try:
        robots_list = [yaml.safe_load(robot.strip()) for robot in
                       robots_str.split(';') if robot.strip()]
        log_msg = f'number_of_robots={len(robots_list)}'
    except yaml.YAMLError as e:
        log_msg = f'Error parsing the robots launch argument: {e}'

    return [LogInfo(msg=[log_msg])]


def generate_robot_actions(name: str = '', pose: dict[str, float] = {}) -> list[GroupAction]:
    """Generate the actions to launch a robot with the given name and pose."""
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    map_yaml_file = LaunchConfiguration('map')
    graph_filepath = LaunchConfiguration('graph')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # Define commands for launching the navigation instances
    group = GroupAction(
            [
                LogInfo(
                    msg=['Launching namespace=', name, ' init_pose=', str(pose),]
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        'namespace': TextSubstitution(text=name),
                        'rviz_config': rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch', 'tb3_simulation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': name,
                        'map': map_yaml_file,
                        'graph': graph_filepath,
                        'use_sim_time': 'True',
                        'params_file': params_file,
                        'autostart': autostart,
                        'use_rviz': 'False',
                        'use_simulator': 'False',
                        'headless': 'False',
                        'use_robot_state_pub': use_robot_state_pub,
                        'x_pose': TextSubstitution(text=str(pose.get('x', 0.0))),
                        'y_pose': TextSubstitution(text=str(pose.get('y', 0.0))),
                        'z_pose': TextSubstitution(text=str(pose.get('z', 0.0))),
                        'roll': TextSubstitution(text=str(pose.get('roll', 0.0))),
                        'pitch': TextSubstitution(text=str(pose.get('pitch', 0.0))),
                        'yaw': TextSubstitution(text=str(pose.get('yaw', 0.0))),
                        'robot_name': TextSubstitution(text=name),
                    }.items(),
                ),
            ]
        )
    return [group]


def generate_launch_description() -> LaunchDescription:
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    Keep general yaml format for pose information.
    ex) robots:='robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: 1.0, y: 1.0, yaw: 1.5707}'
    ex) robots:='robot3={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707};
                 robot4={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}'
    """
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # Simulation settings
    world = LaunchConfiguration('world')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
        description='Full path to world file to load',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml'),
        description='Full path to map file to load',
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(bringup_dir, 'graphs', 'turtlebot3_graph.geojson'),
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_multirobot_params_all.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Automatically startup the stacks',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )

    # Start Gazebo with plugin providing the robot spawning service
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', 'False'], world])
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
    )

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    robots_list = ParseMultiRobotPose('robots').value()

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction(
            [
                LogInfo(
                    msg=[
                        'Launching namespace=',
                        robot_name,
                        ' init_pose=',
                        str(init_pose),
                    ]
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        'namespace': TextSubstitution(text=robot_name),
                        'use_namespace': 'True',
                        'rviz_config': rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch', 'tb3_simulation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': robot_name,
                        'use_namespace': 'True',
                        'map': map_yaml_file,
                        'use_sim_time': 'True',
                        'params_file': params_file,
                        'autostart': autostart,
                        'use_rviz': 'False',
                        'use_simulator': 'False',
                        'headless': 'False',
                        'use_robot_state_pub': use_robot_state_pub,
                        'x_pose': TextSubstitution(text=str(init_pose['x'])),
                        'y_pose': TextSubstitution(text=str(init_pose['y'])),
                        'z_pose': TextSubstitution(text=str(init_pose['z'])),
                        'roll': TextSubstitution(text=str(init_pose['roll'])),
                        'pitch': TextSubstitution(text=str(init_pose['pitch'])),
                        'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                        'robot_name': TextSubstitution(text=robot_name),
                    }.items(),
                ),
            ]
        )

        bringup_cmd_group.append(group)

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(sim_dir)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(world_sdf_xacro)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(remove_temp_sdf_file)

    ld.add_action(LogInfo(msg=['number_of_robots=', str(len(robots_list))]))

    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=['map yaml: ', map_yaml_file])
    )
    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=['params yaml: ', params_file])
    )
    ld.add_action(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=['rviz config file: ', rviz_config_file],
        )
    )
    ld.add_action(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=['using robot state pub: ', use_robot_state_pub],
        )
    )
    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=['autostart: ', autostart])
    )

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
