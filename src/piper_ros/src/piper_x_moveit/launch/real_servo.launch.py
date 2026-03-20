import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    package_dir = get_package_share_directory("piper_x_moveit")

    moveit_config = (
        MoveItConfigsBuilder("piper_x", package_name="piper_x_moveit")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("piper_x_moveit", "config/piper_x_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="can0",
        description="CAN port for the Piper driver.",
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        description="Automatically enable the Piper arm on startup.",
    )
    gripper_exist_arg = DeclareLaunchArgument(
        "gripper_exist",
        default_value="true",
        description="Whether the connected Piper hardware has a gripper.",
    )
    gripper_val_multiple_arg = DeclareLaunchArgument(
        "gripper_val_mutiple",
        default_value="1",
        description="Driver-side multiplier for the gripper command.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the Piper driver.",
    )
    hardware_settle_time_arg = DeclareLaunchArgument(
        "hardware_settle_time",
        default_value="3.0",
        description="Seconds to wait before starting the bridge after the Piper driver.",
    )
    moveit_settle_time_arg = DeclareLaunchArgument(
        "moveit_settle_time",
        default_value="6.0",
        description="Seconds to wait before starting MoveIt and Servo after the Piper driver.",
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, "launch", "rsp.launch.py"))
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "move_group.launch.py")
        )
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "moveit_rviz.launch.py")
        )
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            {"moveit_servo.command_out_topic": "/servo_joint_trajectory"},
            {"moveit_servo.is_primary_planning_scene_monitor": False},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    bridge_node = Node(
        package="piper_servo_bridge",
        executable="piper_servo_bridge",
        name="piper_servo_bridge",
        output="screen",
        parameters=[
            {
                "trajectory_topic": "/servo_joint_trajectory",
                "hardware_command_topic": "joint_ctrl_single",
                "hardware_state_topic": "joint_states_single",
                "moveit_state_topic": "/joint_states",
            }
        ],
    )

    piper_driver = Node(
        package="piper",
        executable="piper_single_ctrl",
        name="piper_ctrl_single_node",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
                "can_port": LaunchConfiguration("can_port"),
                "auto_enable": LaunchConfiguration("auto_enable"),
                "gripper_exist": LaunchConfiguration("gripper_exist"),
                "gripper_val_mutiple": LaunchConfiguration("gripper_val_mutiple"),
            }
        ],
    )

    delayed_bridge = TimerAction(
        period=LaunchConfiguration("hardware_settle_time"),
        actions=[bridge_node],
    )
    delayed_moveit_stack = TimerAction(
        period=LaunchConfiguration("moveit_settle_time"),
        actions=[
            robot_state_publisher,
            move_group,
            rviz_node,
            static_tf,
            servo_node,
        ],
    )

    return LaunchDescription(
        [
            can_port_arg,
            auto_enable_arg,
            gripper_exist_arg,
            gripper_val_multiple_arg,
            log_level_arg,
            hardware_settle_time_arg,
            moveit_settle_time_arg,
            piper_driver,
            delayed_bridge,
            delayed_moveit_stack,
        ]
    )
