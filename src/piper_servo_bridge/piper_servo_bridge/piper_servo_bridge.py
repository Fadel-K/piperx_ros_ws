import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class PiperServoBridge(Node):
    """Bridge MoveIt Servo trajectories to the Piper driver command interface."""

    DEFAULT_ARM_JOINTS = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]

    def __init__(self):
        super().__init__("piper_servo_bridge")

        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("hardware_command_topic", "joint_ctrl_single")
        self.declare_parameter("hardware_state_topic", "joint_states_single")
        self.declare_parameter("moveit_state_topic", "/joint_states")
        self.declare_parameter("arm_joint_names", self.DEFAULT_ARM_JOINTS)

        self.trajectory_topic = self.get_parameter(
            "trajectory_topic"
        ).get_parameter_value().string_value
        self.hardware_command_topic = self.get_parameter(
            "hardware_command_topic"
        ).get_parameter_value().string_value
        self.hardware_state_topic = self.get_parameter(
            "hardware_state_topic"
        ).get_parameter_value().string_value
        self.moveit_state_topic = self.get_parameter(
            "moveit_state_topic"
        ).get_parameter_value().string_value
        self.arm_joint_names = list(
            self.get_parameter("arm_joint_names").get_parameter_value().string_array_value
        )

        self.joint_ctrl_single_publisher = self.create_publisher(
            JointState, self.hardware_command_topic, 10
        )
        self.joint_states_publisher = self.create_publisher(
            JointState, self.moveit_state_topic, 10
        )

        self.joint_trajectory_subscription = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.joint_trajectory_callback,
            10,
        )
        self.joint_states_single_subscription = self.create_subscription(
            JointState,
            self.hardware_state_topic,
            self.joint_states_single_callback,
            10,
        )

        self.get_logger().info(
            "Bridge ready. Trajectory: %s -> %s | Feedback: %s -> %s"
            % (
                self.trajectory_topic,
                self.hardware_command_topic,
                self.hardware_state_topic,
                self.moveit_state_topic,
            )
        )
        self.get_logger().info("Arm joints: %s" % ", ".join(self.arm_joint_names))

    def _filter_joint_values(self, joint_names, values):
        """Return configured arm joints in configured order."""
        if not values:
            return [], []

        index_by_name = {name: idx for idx, name in enumerate(joint_names)}
        filtered_names = []
        filtered_values = []

        for joint_name in self.arm_joint_names:
            idx = index_by_name.get(joint_name)
            if idx is None or idx >= len(values):
                continue
            filtered_values.append(values[idx])
            filtered_names.append(joint_name)

        return filtered_names, filtered_values

    def joint_trajectory_callback(self, msg: JointTrajectory):
        """Convert Servo output to the Piper driver's native JointState command."""
        if not msg.points:
            self.get_logger().warning(
                "Received JointTrajectory with no points; ignoring."
            )
            return

        point = msg.points[0]
        filtered_names, filtered_positions = self._filter_joint_values(
            msg.joint_names, point.positions
        )

        if not filtered_positions:
            self.get_logger().warning(
                "Trajectory did not contain any configured arm joints; ignoring."
            )
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = filtered_names
        joint_state.position = filtered_positions

        self.joint_ctrl_single_publisher.publish(joint_state)

    def joint_states_single_callback(self, msg: JointState):
        """Republish Piper feedback as MoveIt's /joint_states input."""
        filtered_names, filtered_positions = self._filter_joint_values(
            msg.name, msg.position
        )

        if not filtered_positions:
            self.get_logger().warning(
                "Hardware state did not contain any configured arm joints; ignoring."
            )
            return

        moveit_joint_state = JointState()
        moveit_joint_state.header = msg.header
        moveit_joint_state.name = filtered_names
        moveit_joint_state.position = filtered_positions

        if len(msg.velocity) == len(msg.name):
            _, filtered_velocities = self._filter_joint_values(msg.name, msg.velocity)
            moveit_joint_state.velocity = filtered_velocities

        if len(msg.effort) == len(msg.name):
            _, filtered_efforts = self._filter_joint_values(msg.name, msg.effort)
            moveit_joint_state.effort = filtered_efforts

        self.joint_states_publisher.publish(moveit_joint_state)


def main(args=None):
    rclpy.init(args=args)

    piper_servo_bridge = PiperServoBridge()
    rclpy.spin(piper_servo_bridge)

    piper_servo_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
