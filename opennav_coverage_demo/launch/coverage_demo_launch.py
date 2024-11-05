# Copyright (c) 2023 Open Navigation LLC
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

from datetime import datetime
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')
    rviz_config_file = os.path.join(coverage_demo_dir, 'rviz_config.rviz')
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")

    world = os.path.join(coverage_demo_dir, 'blank.world')
    # world = os.path.join(sim_dir, "worlds", "tb3_empty_world.sdf.xacro")
    param_file_path = os.path.join(coverage_demo_dir, 'demo_params.yaml')
    sdf = os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model')
    robot_sdf = os.path.join(sim_dir, "urdf", "gz_waffle.sdf.xacro")

    # declare_simulator_cmd = DeclareLaunchArgument(
    #     'headless', default_value='False', description='Whether to execute gzclient)'
    # )

    # start the simulation
    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so', world],
    #     cwd=[coverage_demo_dir], output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[coverage_demo_dir], output='screen')

    urdf = os.path.join(sim_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        # remappings=remappings,
    )

    world_sdf = tempfile.mktemp(prefix="nav2_", suffix=".sdf")
    # world_sdf_xacro = ExecuteProcess(
    #     cmd=['xacro', '-o', world_sdf, ['headless:=', 'False'], world])
    world_sdf_xacro = ExecuteProcess(cmd=["xacro", "-o", world_sdf, "headless:=false", world])
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_sdf],
        output="screen",
    )

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))])
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    pose = {
        "x": 5.00,
        "y": 5.00,
        "z": 0.10,
        "R": 0.00,
        "P": 0.00,
        "Y": 0.00,
    }

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_dir, "launch", "spawn_tb3.launch.py")),
        launch_arguments={
            "namespace": "",
            # "use_sim_time": "true",
            "robot_name": "turtlebot3_waffle",
            "robot_sdf": robot_sdf,
            "x_pose": str(pose["x"]),
            "y_pose": str(pose["y"]),
            "z_pose": str(pose["z"]),
            "roll": str(pose["R"]),
            "pitch": str(pose["P"]),
            "yaw": str(pose["Y"]),
        }.items(),
    )

    # start_gazebo_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     output='screen',
    #     arguments=[
    #         '-entity', 'tb3',
    #         '-file', sdf,
    #         '-x', '5.0', '-y', '5.0', '-z', '0.10',
    #         '-R', '0.0', '-P', '0.0', '-Y', '0.0'])

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config_file}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    # world->odom transform, no localization. For visualization & controller transform
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    # start the demo task
    demo_cmd = Node(
        package='opennav_coverage_demo',
        executable='demo_coverage',
        emulate_tty=True,
        output='screen')

    # publish initial pose
    # current_time = datetime.now()
    # sec = int(current_time.timestamp())
    # nanosec = int(current_time.microsecond * 1000)  # Convert microseconds to nanoseconds

    message = (
        "{ header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' }, "
        "pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.01 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }, "
        "covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, "
        "0.0685, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0.0685] } }"
    )

    # Define the command to publish to the topic
    publish_cmd_1 = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", message, "--once"],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gz_robot)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(fake_localization_cmd)
    ld.add_action(demo_cmd)
    # ld.add_action(TimerAction(period=1.0, actions=[publish_cmd_1]))
    return ld
