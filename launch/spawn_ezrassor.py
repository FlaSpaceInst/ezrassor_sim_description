"""Launch Gazebo with the specified world file (empty world by default)."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import os


def generate_launch_description():
    """Spawn a new instance of Gazebo Classic with an optional world file."""

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    pkg_ezrassor_sim_description = get_package_share_directory(
        "ezrassor_sim_description"
    )

    ezrassor_model = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "ezrassor.xacro"
    )

    spawn_position_argument = DeclareLaunchArgument(
        "spawn_position",
        default_value="0",
        description="Robot spawn coordinates [x,y]",
    )
    spawn_axes_argument = DeclareLaunchArgument(
        "spawn_axis_argument",
        default_value="0",
        description="Robot spawn axes [roll, pitch, yaw]",
    )
    model_file_argument = DeclareLaunchArgument(
        name="model",
        default_value=ezrassor_model,
        description="Absolute path to robot urdf file",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", LaunchConfiguration("model")]
                )
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ezrassor", "-topic", "/robot_description"],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--state",
            "start",
            "joint_state_controller",
        ],
        output="screen",
    )
    joint_state_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity, on_exit=[load_joint_state_controller]
        )
    )

    load_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--state",
            "start",
            "velocity_controller",
        ],
        output="screen",
    )
    velocity_state_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_velocity_controller],
        )
    )

    return LaunchDescription(
        [
            model_file_argument,
            joint_state_handler,
            velocity_state_handler,
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "worlds/empty.world",
                    "-s",
                    "libgazebo_ros_factory.so",
                ],
                output="screen",
            ),
            robot_state_publisher,
            spawn_entity,
        ]
    )
