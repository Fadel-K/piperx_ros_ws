#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "piper_moveit_cpp",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("piper_moveit_cpp");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
    
  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.4465501790138516;
    msg.orientation.y = 0.43339077089185757;
    msg.orientation.z = -0.5571288881976656;
    msg.orientation.w = 0.5498843326228631;

    msg.position.x = 0.281659;
    msg.position.y = -0.021209;
    msg.position.z = 0.491202;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    const auto& pts = plan.trajectory_.joint_trajectory.points;

    if (!pts.empty())
    {
      const auto& last = pts.back().time_from_start;
      double total_time = last.sec + last.nanosec * 1e-9;

      std::cout << "Number of points: " << pts.size() << std::endl;
      std::cout << "Planned duration: " << total_time << " s" << std::endl;

      for (size_t i = 0; i < pts.size(); ++i)
      {
        double t = pts[i].time_from_start.sec + pts[i].time_from_start.nanosec * 1e-9;
        std::cout << "Point " << i << " time_from_start = " << t << " s" << std::endl;
      }
    }
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}