#include <memory>
#include <vector>
#include <map>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rmw/validate_namespace.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sopias4_msgs/msg/robot.hpp"
#include "sopias4_msgs/msg/namespaces.hpp"
#include "sopias4_msgs/msg/robot_states.hpp"
#include "sopias4_msgs/srv/get_namespaces.hpp"
#include "sopias4_msgs/srv/get_robots.hpp"
#include "sopias4_msgs/srv/get_robot_identity.hpp"
#include "sopias4_msgs/srv/registry_service.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "domain_bridge/domain_bridge.hpp"

class MultiRobotCoordinator : public rclcpp::Node
{
public:
	explicit MultiRobotCoordinator(const std::string &nodeName) : Node(nodeName)
	{
		get_namespaces_service = this->create_service<sopias4_msgs::srv::GetNamespaces>("get_namespaces", std::bind(&MultiRobotCoordinator::get_namespace_callback, this, std::placeholders::_1, std::placeholders::_2));
		get_robot_identity_service = this->create_service<sopias4_msgs::srv::GetRobotIdentity>("get_robot_identity", std::bind(&MultiRobotCoordinator::get_robot_identity_callback, this, std::placeholders::_1, std::placeholders::_2));
		get_robots_service = this->create_service<sopias4_msgs::srv::GetRobots>("get_robots", std::bind(&MultiRobotCoordinator::get_robots_callback, this, std::placeholders::_1, std::placeholders::_2));
		register_service = this->create_service<sopias4_msgs::srv::RegistryService>("register_namespace", std::bind(&MultiRobotCoordinator::register_callback, this, std::placeholders::_1, std::placeholders::_2));
		unregister_service = this->create_service<sopias4_msgs::srv::RegistryService>("unregister_namespace", std::bind(&MultiRobotCoordinator::unregister_callback, this, std::placeholders::_1, std::placeholders::_2));

		publisher_ = this->create_publisher<sopias4_msgs::msg::Namespaces>("registered_namespaces", 10);
		publisher_robot_states_ = this->create_publisher<sopias4_msgs::msg::RobotStates>("robot_states", 10);
		logger.set_level(rclcpp::Logger::Level::Info);

		RCLCPP_INFO(logger, "Started node");

	}

private:
	// All registered namespaces
	std::vector<std::string> registered_namespaces = {};
	// States of all registered robots
	std::vector<sopias4_msgs::msg::Robot> robot_states = {};
	// Pointers of all subscription, mainly so they don't get disallocated. Each pointer is mapped to it's namespace
	std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pose_slam_subscribers_;
	std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pose_amcl_subscribers_;
	std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> plan_subscribers_;
	std::map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> nav_state_subscribers_;
	// logger
	rclcpp::Logger logger = this->get_logger();
	// logger.set_level(rclcpp::Logger::Level::DEBUG);
	// Services
	rclcpp::Service<sopias4_msgs::srv::GetNamespaces>::SharedPtr get_namespaces_service;
	rclcpp::Service<sopias4_msgs::srv::GetRobotIdentity>::SharedPtr get_robot_identity_service;
	rclcpp::Service<sopias4_msgs::srv::GetRobots>::SharedPtr get_robots_service;
	rclcpp::Service<sopias4_msgs::srv::RegistryService>::SharedPtr register_service;
	rclcpp::Service<sopias4_msgs::srv::RegistryService>::SharedPtr unregister_service;
	// Publisher when a new namespace is registered
	rclcpp::Publisher<sopias4_msgs::msg::Namespaces>::SharedPtr publisher_;
	rclcpp::Publisher<sopias4_msgs::msg::RobotStates>::SharedPtr publisher_robot_states_;

	void get_namespace_callback(const sopias4_msgs::srv::GetNamespaces::Request::SharedPtr, sopias4_msgs::srv::GetNamespaces::Response::SharedPtr response)
	{
		RCLCPP_INFO(logger, "Got request to get all registered namespaces");
		response->name_spaces = registered_namespaces;
		return;
	}

	void get_robot_identity_callback(const sopias4_msgs::srv::GetRobotIdentity::Request::SharedPtr request, sopias4_msgs::srv::GetRobotIdentity::Response::SharedPtr response)
	{
		RCLCPP_INFO(logger, "Got request to get the state of the Turtlebot with the namespace %s", request->name_space.c_str());
		// Search through all robot states
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			// Robot is identified by namespace
			if (element->name_space == request->name_space)
			{
				response->robot = *element;
				RCLCPP_DEBUG(logger, "Found state for Turtlebot with namespace %s", request->name_space.c_str());
				break;
			}
		}
		RCLCPP_INFO(logger, "Returned state of Turtlebot with namespace %s", request->name_space.c_str());
		return;
	}

	void get_robots_callback(const sopias4_msgs::srv::GetRobots::Request::SharedPtr, sopias4_msgs::srv::GetRobots::Response::SharedPtr response)
	{
		RCLCPP_INFO(logger, "Got request to return states of all robots");
		response->robots = robot_states;
		return;
	}

	void register_callback(const sopias4_msgs::srv::RegistryService::Request::SharedPtr request, sopias4_msgs::srv::RegistryService::Response::SharedPtr response)
	{
		RCLCPP_INFO(logger, "Got request to register namespace %s", request->name_space.c_str());
		// --- Validate namespace ---
		int validation_result;
		size_t invalid_index;

		rmw_ret_t rmw_ret =
			rmw_validate_namespace(request->name_space.c_str(), &validation_result, &invalid_index);

		RCLCPP_DEBUG(logger, "Validate namespace %s: Validate if validation itself failed", request->name_space.c_str());
		// Check if validation itself failed
		if (rmw_ret != RMW_RET_OK)
		{
			if (rmw_ret == RMW_RET_INVALID_ARGUMENT)
			{
				rclcpp::exceptions::throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate subnode namespace");
			}
			rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, "failed to validate subnode namespace");
		}
		RCLCPP_DEBUG(logger, "Validate namespace %s: Validate if namespace is legal", request->name_space.c_str());
		// Check result from validatiom
		if (validation_result != RMW_NAMESPACE_VALID)
		{
			RCLCPP_WARN(logger, "Namespace %s isn't legal: Validation response %d", request->name_space.c_str(), validation_result);
			response->statuscode = sopias4_msgs::srv::RegistryService::Response::ILLEGAL_NAMESPACE_ERROR;
			return;
		}
		RCLCPP_DEBUG(logger, "Validate namespace %s: Validate if namespace isn't already registered", request->name_space.c_str());
		// --- Check if namespace is already registered --
		if (std::find(registered_namespaces.begin(), registered_namespaces.end(), request->name_space) != registered_namespaces.end())
		{
			response->statuscode = sopias4_msgs::srv::RegistryService::Response::COLLISION_ERROR;
			return;
		}
		RCLCPP_DEBUG(logger, "Validated namespace %s", request->name_space.c_str());

		RCLCPP_DEBUG(logger, "Adding subscriptions for namespace %s", request->name_space.c_str());
		// --- Register namespace ---
		registered_namespaces.push_back(request->name_space);
		response->statuscode = sopias4_msgs::srv::RegistryService::Response::SUCCESS;

		// Use a callback factory to pass a second argument to callback function (known bug in ros2, so this workaround is needed)
		std::function<void(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)> callback_fcn = std::bind(&MultiRobotCoordinator::set_pose_callback, this, std::placeholders::_1, request->name_space);
		std::function<void(nav_msgs::msg::Path::SharedPtr msg)> callback_fcn_path = std::bind(&MultiRobotCoordinator::set_robot_path_sub_callback, this, std::placeholders::_1, request->name_space);
		std::function<void(std_msgs::msg::Bool::SharedPtr msg)> callback_fcn_nav_state = std::bind(&MultiRobotCoordinator::nav_state_sub_callback, this, std::placeholders::_1, request->name_space);

		std::string topic_name = request->name_space + std::string("/amcl_pose");
		pose_amcl_subscribers_[request -> name_space] = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, rclcpp::SensorDataQoS(), callback_fcn);
		topic_name = request->name_space + std::string("/pose");
		pose_slam_subscribers_[request -> name_space] = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, rclcpp::SensorDataQoS(), callback_fcn);
		topic_name = request->name_space + std::string("/plan");
		plan_subscribers_[request -> name_space]= this->create_subscription<nav_msgs::msg::Path>(topic_name, rclcpp::SensorDataQoS(), callback_fcn_path);
		topic_name = request->name_space + std::string("/is_navigating");
		nav_state_subscribers_[request -> name_space]= this->create_subscription<std_msgs::msg::Bool>(topic_name, rclcpp::SensorDataQoS(), callback_fcn_nav_state);

		RCLCPP_DEBUG(logger, "Added subscriptions for namespace %s", request->name_space.c_str());


		RCLCPP_DEBUG(logger, "Adding robot state for namespace %s", request->name_space.c_str());
		// --- Create state for registered robot --
		sopias4_msgs::msg::Robot robot_state = sopias4_msgs::msg::Robot();
		robot_state.name_space = request->name_space;
		robot_states.push_back(robot_state);
		RCLCPP_DEBUG(logger, "Added robot state for namespace %s", request->name_space.c_str());

		//  Publish registered namespaces
		sopias4_msgs::msg::Namespaces msg = sopias4_msgs::msg::Namespaces();
		msg.name_spaces = registered_namespaces;
		publisher_->publish(msg);
		//  Publish registered robot_states
		publish_robot_states();

		RCLCPP_INFO(logger, "Successfully registered namespace %s", request->name_space.c_str());
	}

	void unregister_callback(const sopias4_msgs::srv::RegistryService::Request::SharedPtr request, sopias4_msgs::srv::RegistryService::Response::SharedPtr response)
	{
		RCLCPP_INFO(logger, "Got request to unregister namespace %s", request->name_space.c_str());
		response->statuscode = sopias4_msgs::srv::RegistryService::Response::UNKNOWN_ERROR;
		// --- Delete namespace --
		for (auto element = registered_namespaces.begin(); element != registered_namespaces.end(); ++element)
		{
			response->statuscode = sopias4_msgs::srv::RegistryService::Response::NS_NOT_FOUND;
			if (*element == request->name_space)
			{
				registered_namespaces.erase(element);
				response->statuscode = sopias4_msgs::srv::RegistryService::Response::SUCCESS;
				break;
			}
		}
		RCLCPP_DEBUG(logger, "Deleted namespace %s from the internal list", request->name_space.c_str());

		// --- Delete state of robot ---
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			if (element->name_space == request->name_space)
			{
				robot_states.erase(element);
				response->statuscode = sopias4_msgs::srv::RegistryService::Response::SUCCESS;
				break;
			}
		}
		RCLCPP_DEBUG(logger, "Deleted state of robot with namespace %s", request->name_space.c_str());

		// ---  Unallocate subscribers ---
		pose_amcl_subscribers_.erase(request->name_space);
		pose_slam_subscribers_.erase(request -> name_space);
		plan_subscribers_.erase(request -> name_space);

		RCLCPP_DEBUG(logger, "Unallocated subscribers with namespace %s", request->name_space.c_str());

		//  Publish registered namespaces
		publish_robot_states();

		RCLCPP_INFO(logger, "Successfully unregistered namespace %s", request->name_space.c_str());
	}

	void set_robot_path_sub_callback(const nav_msgs::msg::Path::SharedPtr path, std::string name_space)
	{
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			if (element->name_space == name_space)
			{
				element->nav_path = *path;
				publish_robot_states();
				RCLCPP_DEBUG(logger, "Successfully set path for robot with namespace %s", name_space.c_str());
				return;
			}
		}
	}

	void set_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose, const std::string name_space)
	{
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			if (element->name_space == name_space)
			{
				element->pose = *pose;
				// Publish updated state of robots
				publish_robot_states();
				return;
			}
		}
	}

	void nav_state_sub_callback(const std_msgs::msg::Bool::SharedPtr is_navigating, const std::string name_space)
	{
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			if (element->name_space == name_space)
			{
				element->is_navigating = is_navigating->data;
				// Publish updated state of robots
				publish_robot_states();
				return;
			}
		}
	}

	void publish_robot_states()
	{
		// Publish updated state of robots
		sopias4_msgs::msg::RobotStates msg = sopias4_msgs::msg::RobotStates();
		msg.robot_states = robot_states;
		publisher_robot_states_->publish(msg);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor=  rclcpp::executors::MultiThreadedExecutor();

	// Create node
	MultiRobotCoordinator::SharedPtr node = std::make_shared<MultiRobotCoordinator>("multi_robot_coordinator");

	// --- Bridge topics ---
	domain_bridge::DomainBridge domain_bridge;
	// map from fleetbroker
	domain_bridge.bridge_topic("/map","nav_msgs/msg/OccupancyGrid",0,1);
	domain_bridge.bridge_topic("/map","nav_msgs/msg/OccupancyGrid",0,2);
	domain_bridge.bridge_topic("/map","nav_msgs/msg/OccupancyGrid",0,3);
	// States of robots
	domain_bridge.bridge_topic("/robot_states","sopias4_msgs/msg/RobotStates",0,1);
	domain_bridge.bridge_topic("/robot_states","sopias4_msgs/msg/RobotStates",0,2);
	domain_bridge.bridge_topic("/robot_states","sopias4_msgs/msg/RobotStates",0,3);
	// amcl_pose
	domain_bridge.bridge_topic("/turtle1/amcl_pose","geometry_msgs/msg/PoseWithCovarianceStamped",1,0);
	domain_bridge.bridge_topic("/turtle2/amcl_pose","geometry_msgs/msg/PoseWithCovarianceStamped",2,0);
	domain_bridge.bridge_topic("/turtle3/amcl_pose","geometry_msgs/msg/PoseWithCovarianceStamped",3,0);
	// slam pose
	domain_bridge.bridge_topic("/turtle1/pose","geometry_msgs/msg/PoseWithCovarianceStamped",1,0);
	domain_bridge.bridge_topic("/turtle2/pose","geometry_msgs/msg/PoseWithCovarianceStamped",2,0);
	domain_bridge.bridge_topic("/turtle3/pose","geometry_msgs/msg/PoseWithCovarianceStamped",3,0);
	// planned global path
	domain_bridge.bridge_topic("/turtle1/plan","nav_msgs/msg/Path",1,0);
	domain_bridge.bridge_topic("/turtle2/plan","nav_msgs/msg/Path",2,0);
	domain_bridge.bridge_topic("/turtle3/plan","nav_msgs/msg/Path",3,0);
	// Is navigating topic
	domain_bridge.bridge_topic("/turtle1/is_navigation","std_msgs/msg/Bool",1,0);
	domain_bridge.bridge_topic("/turtle2/is_navigation","std_msgs/msg/Bool",2,0);
	domain_bridge.bridge_topic("/turtle3/is_navigation","std_msgs/msg/Bool",3,0);
	// Map from slam
	domain_bridge.bridge_topic("/turtle1/map","nav_msgs/msg/OccupancyGrid",1,0);
	domain_bridge.bridge_topic("/turtle2/map","nav_msgs/msg/OccupancyGrid",2,0);
	domain_bridge.bridge_topic("/turtle3/map","nav_msgs/msg/OccupancyGrid",3,0);
	// Services
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/register_namespace",0,1);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/register_namespace",0,2);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/register_namespace",0,3);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/unregister_namespace",0,1);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/unregister_namespace",0,2);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/unregister_namespace",0,3);
	domain_bridge.bridge_service<nav2_msgs::srv::SaveMap>("/map_saver/save_map",0,1);
	domain_bridge.bridge_service<nav2_msgs::srv::SaveMap>("/map_saver/save_map",0,2);
	domain_bridge.bridge_service<nav2_msgs::srv::SaveMap>("/map_saver/save_map",0,3);
	domain_bridge.add_to_executor(executor);

	// Run node
	executor.add_node(node);
	executor.spin();

	// Shutdown node	
	rclcpp::shutdown();

	return 0;
}