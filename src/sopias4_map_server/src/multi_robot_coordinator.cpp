#include <memory>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rmw/validate_namespace.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sopias4_msgs/msg/robot.hpp"
#include "sopias4_msgs/msg/namespaces.hpp"
#include "sopias4_msgs/msg/robot_states.hpp"
#include "sopias4_msgs/srv/get_namespaces.hpp"
#include "sopias4_msgs/srv/get_robots.hpp"
#include "sopias4_msgs/srv/get_robot_identity.hpp"
#include "sopias4_msgs/srv/set_robot_path.hpp"
#include "sopias4_msgs/srv/registry_service.hpp"

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
		set_robot_path_service = this->create_service<sopias4_msgs::srv::SetRobotPath>("set_robot_path", std::bind(&MultiRobotCoordinator::set_robot_path_callback, this, std::placeholders::_1, std::placeholders::_2));

		publisher_ = this->create_publisher<sopias4_msgs::msg::Namespaces>("registered_namespaces", 10);
		publisher_robot_states_ = this->create_publisher<sopias4_msgs::msg::RobotStates>("robot_states", 10);
		RCLCPP_INFO(logger, "Started node");
		logger.set_level(rclcpp::Logger::Level::Info);
	}

private:
	// All registered namespaces
	std::vector<std::string> registered_namespaces = {};
	// States of all registered robots
	std::vector<sopias4_msgs::msg::Robot> robot_states = {};
	// logger
	rclcpp::Logger logger = this->get_logger();
	// logger.set_level(rclcpp::Logger::Level::DEBUG);

	// Services
	rclcpp::Service<sopias4_msgs::srv::GetNamespaces>::SharedPtr get_namespaces_service;
	rclcpp::Service<sopias4_msgs::srv::GetRobotIdentity>::SharedPtr get_robot_identity_service;
	rclcpp::Service<sopias4_msgs::srv::GetRobots>::SharedPtr get_robots_service;
	rclcpp::Service<sopias4_msgs::srv::RegistryService>::SharedPtr register_service;
	rclcpp::Service<sopias4_msgs::srv::RegistryService>::SharedPtr unregister_service;
	rclcpp::Service<sopias4_msgs::srv::SetRobotPath>::SharedPtr set_robot_path_service;
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
				RCLCPP_DEBUG(logger, "Found state for T urtlebot with namespace %s", request->name_space.c_str());
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

		// --- Register namespace ---
		registered_namespaces.push_back(request->name_space);
		response->statuscode = sopias4_msgs::srv::RegistryService::Response::SUCCESS;

		// Use a callback factory to pass a second argument to callback function (known bug in ros2, so this workaround is needed)
		std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)> callback_fcn = std::bind(&MultiRobotCoordinator::set_pose_callback, this, std::placeholders::_1, request->name_space);
		std::function<void(const nav_msgs::msg::Path::SharedPtr msg)> callback_fcn_path = std::bind(&MultiRobotCoordinator::set_robot_path_sub_callback, this, std::placeholders::_1, request->name_space);

		this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(request->name_space + std::string("/amcl_pose"), 10, callback_fcn);
		this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(request->name_space + std::string("/pose"), 10, callback_fcn);
		this->create_subscription<nav_msgs::msg::Path>(request->name_space + std::string("/plan"), 10, callback_fcn_path);

		// --- Create state for registered robot --
		sopias4_msgs::msg::Robot robot_state = sopias4_msgs::msg::Robot();
		robot_state.name_space = request->name_space;
		robot_states.push_back(robot_state);

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

		//  Publish registered namespaces
		publish_robot_states();

		RCLCPP_INFO(logger, "Successfully unregistered namespace %s", request->name_space.c_str());
		return;
	}

	void set_robot_path_callback(const sopias4_msgs::srv::SetRobotPath::Request::SharedPtr request, sopias4_msgs::srv::SetRobotPath::Response::SharedPtr response)
	{
		RCLCPP_INFO(logger, "Got request to set path for robot with namespace %s", request->name_space.c_str());
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			response->statuscode = sopias4_msgs::srv::SetRobotPath::Response::ROBOT_NOT_FOUND;
			if (element->name_space == request->name_space)
			{
				element->nav_path = request->path;
				response->statuscode = sopias4_msgs::srv::SetRobotPath::Response::SUCCESS;
				break;
			}
		}

		// Publish updated state of robots
		publish_robot_states();
		RCLCPP_INFO(logger, "Successfully set path for robot with namespace %s", request->name_space.c_str());
		return;
	}

	void set_robot_path_sub_callback(const nav_msgs::msg::Path::SharedPtr path, std::string name_space)
	{
		RCLCPP_INFO(logger, "Got request to set path for robot with namespace %s", name_space.c_str());
		for (auto element = robot_states.begin(); element != robot_states.end(); ++element)
		{
			if (element->name_space == name_space)
			{
				element->nav_path = *path;
				break;
			}
		}

		publish_robot_states();
		RCLCPP_INFO(logger, "Successfully set path for robot with namespace %s", name_space.c_str());
		return;
	}

	void set_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose, std::string name_space)
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

	// Create node
	auto node = std::make_shared<MultiRobotCoordinator>("multi_robot_coordinator");

	// Run node
	rclcpp::spin(node);

	// Shutdown node
	rclcpp::shutdown();

	return 0;
}