#include <cmath>
#include <string>
#include <memory>
#include <iomanip>
#include <iostream>
#include <ctime>
#include <sstream>
#include "nav2_util/node_utils.hpp"
#include "sopias4_framework/plugin_bridge_planner.hpp"
#include "sopias4_framework/msgs_utils.hpp"
#include "sopias4_msgs/srv/create_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <json/json.h>
#include <fstream>
#include <string>
#include "yaml-cpp/yaml.h"

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

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".plugin_name", rclcpp::ParameterValue("global_planner"));
    node_->get_parameter(name + ".plugin_name", plugin_name_);
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".save_costmaps", rclcpp::ParameterValue(false));
    node_->get_parameter(name + ".save_costmaps", save_costmaps_);
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".save_path", rclcpp::ParameterValue("/home/ws/src/sopias4_framework/assets/global_costmaps/"));
    node_->get_parameter(name + ".save_path", save_path_);

    // Service client node
    service_node_ = std::make_shared<rclcpp::Node>("_planner_bridge_service_node_" + plugin_name_);
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
    client_ = service_node_->create_client<sopias4_msgs::srv::CreatePlan>(plugin_name_ + "/create_plan");
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
    global_path.header.frame_id = global_frame_;
    global_path.header.stamp = node_->now();

    if (!client_->service_is_ready())
    {
      RCLCPP_ERROR(node_->get_logger(), "No service is online where the planner bridge can send an service request to");
      return global_path;
    }

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
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only except goal position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    sopias4_msgs::srv::CreatePlan::Request::SharedPtr request = std::make_shared<sopias4_msgs::srv::CreatePlan::Request>();
    // Generate service request
    request->start = start;
    request->goal = goal;
    request->costmap = sopias4_framework::tools::costmap_2_costmap_msg(costmap_, global_frame_);

    if (save_costmaps_)
    {
      PlannerBridge::save_request(request, save_path_, costmap_);
    }

    auto future = client_->async_send_request(request);
    auto return_code = rclcpp::spin_until_future_complete(service_node_, future);

    if (return_code == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_DEBUG(
          node_->get_logger(), "Received plan from implemented plugin");
      global_path.poses = future.get()->global_path.poses;
      return global_path;
    }
    else
    {
      return global_path;
    }
  }

  void PlannerBridge::save_request(sopias4_msgs::srv::CreatePlan::Request::SharedPtr request, std::string save_path, nav2_costmap_2d::Costmap2D *costmap)
  {
    // Get timestamp

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%y_%m_%y__%H_%H_%S");

    // Save request to Json
    {
      Json::Value json_object;
      // Start
      json_object["start"]["pose"]["position"]["x"] = request->start.pose.position.x;
      json_object["start"]["pose"]["position"]["y"] = request->start.pose.position.y;
      json_object["start"]["pose"]["orientation"]["y"] = request->start.pose.orientation.y;
      json_object["start"]["pose"]["orientation"]["w"] = request->start.pose.orientation.w;
      json_object["start"]["header"]["frame_id"] = request->start.header.frame_id;
      // goal
      json_object["goal"]["pose"]["position"]["x"] = request->goal.pose.position.x;
      json_object["goal"]["pose"]["position"]["y"] = request->goal.pose.position.y;
      json_object["goal"]["pose"]["orientation"]["y"] = request->goal.pose.orientation.y;
      json_object["goal"]["pose"]["orientation"]["w"] = request->goal.pose.orientation.w;
      json_object["goal"]["header"]["frame_id"] = request->goal.header.frame_id;
      // Costmap value
      json_object["costmap"]["info"]["height"] = request->costmap.info.height;
      json_object["costmap"]["info"]["width"] = request->costmap.info.width;
      json_object["costmap"]["info"]["resolution"] = request->costmap.info.resolution;
      json_object["costmap"]["info"]["origin"]["position"]["x"] = request->costmap.info.origin.position.x;
      json_object["costmap"]["info"]["origin"]["position"]["y"] = request->costmap.info.origin.position.y;
      json_object["costmap"]["info"]["origin"]["orientation"]["y"] = request->costmap.info.origin.orientation.y;
      json_object["costmap"]["info"]["origin"]["orientation"]["w"] = request->costmap.info.origin.orientation.w;
      json_object["costmap"]["header"]["frame_id"] = request->costmap.header.frame_id;
      Json::Value data(Json::arrayValue);
      for (unsigned int i = 0; i < request->costmap.data.size(); i++)
      {
        data.append(Json::Value(request->costmap.data[i]));
      }
      json_object["costmap"]["data"] = data;

      Json::StreamWriterBuilder builder;
      builder["commentStyle"] = "None";
      builder["indentation"] = "   ";
      std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
      std::ofstream outputFileStream(save_path + "global_costmap_" + oss.str() + ".json");
      writer->write(json_object, &outputFileStream);
    }
    
    // Save costmap as a map
    // {
    //   costmap->saveMap(save_path + "global_costmap_" + oss.str() + ".pgm");
    //   YAML::Emitter e;
    //   e << YAML::Precision(3);
    //   e << YAML::BeginMap;
    //   e << YAML::Key << "image" << YAML::Value << save_path + "global_costmap_" + oss.str() + ".pgm";
    //   e << YAML::Key << "mode" << YAML::Value << "trinary";
    //   e << YAML::Key << "resolution" << YAML::Value << request->costmap.info.resolution;
    //   geometry_msgs::msg::Quaternion orientation = request->costmap.info.origin.orientation;
    //   tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    //   double yaw, pitch, roll;
    //   mat.getEulerYPR(yaw, pitch, roll);
    //   e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << request->costmap.info.origin.position.x << request->costmap.info.origin.position.y << 0 << YAML::EndSeq;
    //   e << YAML::Key << "negate" << YAML::Value << 0;
    //   e << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    //   e << YAML::Key << "free_thresh" << YAML::Value << 0.2;

    //   if (!e.good())
    //   {
    //     std::cout << "[WARN] [map_io]: YAML writer failed with an error " << e.GetLastError() << ". The map metadata may be invalid." << std::endl;
    //   }
    //   std::ofstream(save_path + "global_costmap_" + oss.str() + ".yaml") << e.c_str();
    // }
  }

} // namespace plugin_bridge

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plugin_bridges::PlannerBridge, nav2_core::GlobalPlanner)