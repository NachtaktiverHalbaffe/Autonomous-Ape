#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "sopias4_framework/msgs_utils.hpp"

#include "sopias4_framework/plugin_bridge_controller.hpp"
#include "sopias4_msgs/srv/compute_velocity_commands.hpp"

using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace plugin_bridges
{

  /**
   * Find element in iterator with the minimum calculated value
   */
  template <typename Iter, typename Getter>
  Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end)
    {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it)
    {
      auto comp = getCompareVal(*it);
      if (comp < lowest)
      {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }

  void ControllerBridge::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros)
  {
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(node, name_ + "plugin_name", rclcpp::ParameterValue("local_layer"));
    
    node->get_parameter(name_ + "plugin_name", plugin_name_);
    double transform_tolerance;
    node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    // Service client node
    service_node_ = std::make_shared<rclcpp::Node>("_planner_bridge_service_node_" + plugin_name_);
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  }

  void ControllerBridge::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type plugin_bridges::ControllerBridge",
        name_.c_str());
    global_pub_.reset();
  }

  void ControllerBridge::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type plugin_bridges::ControllerBridge\"  %s",
        name_.c_str());

    global_pub_->on_activate();
    client_compute_vel_ = service_node_->create_client<sopias4_msgs::srv::ComputeVelocityCommands>(plugin_name_ + "/compute_velocity_commands");
  }

  void ControllerBridge::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type plugin_bridges::ControllerBridge\"  %s",
        name_.c_str());
    global_pub_->on_deactivate();
  }

  geometry_msgs::msg::TwistStamped ControllerBridge::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity)
  {
    auto transformed_plan = transformGlobalPlan(pose);

    // Find the first pose which is at a distance greater than the specified lookahed distance
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
        { return hypot(ps.pose.position.x, ps.pose.position.y) >= 0.4; });

    // If the last pose is still within lookahed distance, take the last pose
    if (goal_pose_it == transformed_plan.poses.end())
    {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    auto goal_pose = goal_pose_it->pose;

    auto request = std::make_shared<sopias4_msgs::srv::ComputeVelocityCommands::Request>();
    request->current_pose = pose;
    request->goal_pose = goal_pose;
    request->current_vel_cmd = velocity;
    request->local_costmap = sopias4_framework::tools::costmap_2_costmap_msg(costmap_ros_->getCostmap());

    auto future = client_compute_vel_->async_send_request(request);
    auto return_code = rclcpp::spin_until_future_complete(service_node_, future);

    // Create and publish a TwistStamped message with the desired velocity
    if (return_code == rclcpp::FutureReturnCode::SUCCESS)
    {
      return future.get()->cmd_vel;
    }
    else
    {
      return geometry_msgs::msg::TwistStamped();
    }
  }

  void ControllerBridge::setPlan(const nav_msgs::msg::Path &path)
  {
    global_pub_->publish(path);
    global_plan_ = path;
  }

  nav_msgs::msg::Path
  ControllerBridge::transformGlobalPlan(
      const geometry_msgs::msg::PoseStamped &pose)
  {
    // Original implementation taken from nav2_dwb_controller

    if (global_plan_.poses.empty())
    {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
            tf_, global_plan_.header.frame_id, pose,
            robot_pose, transform_tolerance_))
    {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                            costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
            {
              return euclidean_distance(robot_pose, ps);
            });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto &global_plan_pose)
        {
          return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
    {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
          tf_, costmap_ros_->getBaseFrameID(),
          stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty())
    {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  bool ControllerBridge::transformPose(
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::string frame,
      const geometry_msgs::msg::PoseStamped &in_pose,
      geometry_msgs::msg::PoseStamped &out_pose,
      const rclcpp::Duration &transform_tolerance) const
  {
    // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    if (in_pose.header.frame_id == frame)
    {
      out_pose = in_pose;
      return true;
    }

    try
    {
      tf->transform(in_pose, out_pose, frame);
      return true;
    }
    catch (tf2::ExtrapolationException &ex)
    {
      auto transform = tf->lookupTransform(
          frame,
          in_pose.header.frame_id,
          tf2::TimePointZero);
      if (
          (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
          transform_tolerance)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Transform data too old when converting from %s to %s",
            in_pose.header.frame_id.c_str(),
            frame.c_str());
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Data time: %ds %uns, Transform time: %ds %uns",
            in_pose.header.stamp.sec,
            in_pose.header.stamp.nanosec,
            transform.header.stamp.sec,
            transform.header.stamp.nanosec);
        return false;
      }
      else
      {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Exception in transformPose: %s",
          ex.what());
      return false;
    }
    return false;
  }
}
// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(plugin_bridges::ControllerBridge, nav2_core::Controller)