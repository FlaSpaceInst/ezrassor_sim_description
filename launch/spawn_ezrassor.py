"""Launch Gazebo with the specified world file (empty world by default)."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.opaque_function import OpaqueFunction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, Command
import os


def __spawn_robot(context, *args, **kwargs):
    """Returns the nodes for spawning a unique robot in Gazebo.

    Must be called from an OpaqueFunction() for a LaunchDescription.

    e.g., return LaunchDescription([OpaqueFunction(__spawn_robot)])

    """

    # The context comes from __spawn_robot being called within an OpaqueFunction and
    # allows us to get a launch argument before nodes get created.
    robot_name = LaunchConfiguration("robot_name").perform(context)
    if robot_name[0] == "/":
        robot_name = robot_name[1:]

    # Since we are building the namespace off of the robot name, ensure
    # it has a leading /
    namespace = robot_name
    if namespace[0] != "/":
        namespace = f"/{namespace}"

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=namespace,
        arguments=[
            "-entity",
            robot_name,
            "-robot_namespace",
            namespace,
            "-topic",
            f"{namespace}/robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
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
    return [joint_state_handler, velocity_state_handler, spawn_entity]


def generate_launch_description():
    """Spawn a new instance of Gazebo Classic with an optional world file."""

    pkg_ezrassor_sim_description = get_package_share_directory(
        "ezrassor_sim_description"
    )

    robot_name_argument = DeclareLaunchArgument(
        "robot_name",
        default_value="ezrassor",
        description="Entity name and namespace for robot spawn",
    )

    # spawn_entity.py takes in these arguments separately,
    # doing the same here for consistency.
    x_position_argument = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="X position for robot spawn",
    )
    y_position_argument = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Y position for robot spawn",
    )
    z_position_argument = DeclareLaunchArgument(
        "z",
        default_value="0.0",
        description="Z position for robot spawn",
    )
    r_axis_argument = DeclareLaunchArgument(
        "R",
        default_value="0.0",
        description="Roll angle for robot spawn",
    )
    p_axis_argument = DeclareLaunchArgument(
        "P",
        default_value="0.0",
        description="Pitch angle for robot spawn",
    )
    y_axis_argument = DeclareLaunchArgument(
        "Y",
        default_value="0.0",
        description="Yaw angle for robot spawn",
    )

    ezrassor_model = os.path.join(
        pkg_ezrassor_sim_description, "urdf", "ezrassor.xacro"
    )
    model_file_argument = DeclareLaunchArgument(
        name="model",
        default_value=ezrassor_model,
        description="Absolute path to robot urdf file",
    )

    # State publisher needs to be tied to the unique instance of the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("robot_name"),
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

    # Note that this package WILL NOT start Gazebo
    # instead, when this launch file is executed it will wait for /spawn_entity
    # to be available. This will automatically be available after Gazebo is launched
    # from ROS2.
    return LaunchDescription(
        [
            robot_name_argument,
            x_position_argument,
            y_position_argument,
            z_position_argument,
            r_axis_argument,
            p_axis_argument,
            y_axis_argument,
            model_file_argument,
            robot_state_publisher,
            OpaqueFunction(function=__spawn_robot),
            # OpaqueFunction allows for getting arguments before nodes get launched. It
            # is needed to set the namespace for a node since it only accepts a string.
        ]
    )
