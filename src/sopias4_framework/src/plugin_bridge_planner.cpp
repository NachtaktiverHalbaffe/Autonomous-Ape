#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "sopias4_framework/plugin_bridge_planner.hpp"
#include "sopias4_framework/msgs_utils.hpp"
#include "sopias4_msgs/srv/create_plan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plugin_bridges
{

  void PlannerBridge::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // Plugin specific stuff
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    // Service client

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".plugin_name", rclcpp::ParameterValue("global_planner"));
    node_->get_parameter(name + ".plugin_name", plugin_name_);
  }

  void PlannerBridge::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type PlannerPluginBridge",
        name_.c_str());
  }

  void PlannerBridge::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type PlannerPluginBridge",
        name_.c_str());
    client_ = node_->create_client<sopias4_msgs::srv::CreatePlan>(plugin_name_ + "/create_plan");
  }

  void PlannerBridge::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type PlannerPluginBridge",
        name_.c_str());
  }

  nav_msgs::msg::Path PlannerBridge::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only except start position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner will only except goal position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    sopias4_msgs::srv::CreatePlan::Request::SharedPtr request = std::make_shared<sopias4_msgs::srv::CreatePlan::Request>();
    // Generate service request
    request->start = start;
    request->goal = goal;
    request->costmap = sopias4_framework::tools::costmap_2_costmap_msg(costmap_);

    auto future = client_->async_send_request(request);
    auto return_code = rclcpp::spin_until_future_complete(node_, future);

    if (return_code == rclcpp::FutureReturnCode::SUCCESS)
    {
      return future.get()->global_path;
    }
    else
    {
      return global_path;
    }
  }

} // namespace plugin_bridge

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plugin_bridges::PlannerBridge, nav2_core::GlobalPlanner)