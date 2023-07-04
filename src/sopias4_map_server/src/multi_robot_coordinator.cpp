#include <memory>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rmw/validate_namespace.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sopias4_msgs/srv/get_namespaces.hpp"
#include "sopias4_msgs/srv/get_robots.hpp"
#include "sopias4_msgs/srv/get_robot_identity.hpp"
#include "sopias4_msgs/srv/register.hpp"
#include "sopias4_msgs/srv/unregister.hpp"
#include "sopias4_msgs/srv/set_robot_path.hpp"
#include "sopias4_msgs/msg/robot.hpp"



class MultiRobotCoordinator : public rclcpp:: Node {
public:
explicit MultiRobotCoordinator(const std::string & nodeName) : Node(nodeName){
	get_namespaces_service =create_service<sopias4_msgs::srv::GetNamespaces>("get_namespaces", std::bind(&MultiRobotCoordinator::get_namespace_callback, this, std::placeholders::_1, std::placeholders::_2));
	get_robot_identity_service= create_service<sopias4_msgs::srv::GetRobotIdentity>("get_robot_identity", std::bind(&MultiRobotCoordinator::get_robot_identity_callback, this,std::placeholders::_1, std::placeholders::_2));
	get_robots_service = create_service<sopias4_msgs::srv::GetRobots>("get_robots", std::bind(&MultiRobotCoordinator::get_robots_callback, this, std::placeholders::_1, std::placeholders::_2) );
	register_service = create_service<sopias4_msgs::srv::Register>("register_namespace", std::bind(&MultiRobotCoordinator::register_callback, this, std::placeholders::_1, std::placeholders::_2));
	unregister_service = create_service<sopias4_msgs::srv::Unregister>("unregister_namespace", std::bind(&MultiRobotCoordinator::unregister_callback, this, std::placeholders::_1, std::placeholders::_2));
	set_robot_path_service = create_service<sopias4_msgs::srv::SetRobotPath>("set_robot_path", std::bind(&MultiRobotCoordinator::set_robot_path_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Started node");
	logger.set_level(rclcpp::Logger::Level::Debug);
}
private:
// All registered namespaces
std::vector<std::string> registered_namespaces = {};
// States of all registered robots
std::vector<sopias4_msgs::msg::Robot> robot_states = {};
// logger
rclcpp::Logger logger = this -> get_logger();
// logger.set_level(rclcpp::Logger::Level::DEBUG);

// Services
rclcpp::Service<sopias4_msgs::srv::GetNamespaces>::SharedPtr get_namespaces_service;
rclcpp::Service<sopias4_msgs::srv::GetRobotIdentity>::SharedPtr get_robot_identity_service;
rclcpp::Service<sopias4_msgs::srv::GetRobots>::SharedPtr get_robots_service;
rclcpp::Service<sopias4_msgs::srv::Register>::SharedPtr register_service;
rclcpp::Service<sopias4_msgs::srv::Unregister>::SharedPtr unregister_service;
rclcpp::Service<sopias4_msgs::srv::SetRobotPath>::SharedPtr set_robot_path_service;

void get_namespace_callback(const sopias4_msgs::srv::GetNamespaces::Request::SharedPtr request, sopias4_msgs::srv::GetNamespaces::Response::SharedPtr response ){
	RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Got request to get all registered namespaces");
	response->name_spaces = registered_namespaces;
	return;
}

void get_robot_identity_callback(const sopias4_msgs::srv::GetRobotIdentity::Request::SharedPtr request, sopias4_msgs::srv::GetRobotIdentity::Response::SharedPtr response){
    RCLCPP_INFO(logger,"[MULTI ROBOT COORDINATOR] Got request to get the state of the Turtlebot with the namespace %s", request -> name_space.c_str());
	// Search through all robot states
	for(auto element = robot_states.begin(); element != robot_states.end(); ++element) {
		// Robot is identified by namespace
		if( element->name_space == request->name_space) {
			response->robot = *element;
            RCLCPP_DEBUG(logger, "[MULTI ROBOT COORDINATOR] Found state for T urtlebot with namespace %s", request -> name_space.c_str());
			break;
		}
	}
    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Returned state of Turtlebot with namespace %s", request -> name_space.c_str());
	return;
}

void get_robots_callback(const sopias4_msgs::srv::GetRobots::Request::SharedPtr request, sopias4_msgs::srv::GetRobots::Response::SharedPtr response){
    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Got request to return states of all robots");
	response->robots = robot_states;
	return;
}

void register_callback(const sopias4_msgs::srv::Register::Request::SharedPtr request, sopias4_msgs::srv::Register::Response::SharedPtr response){
	RCLCPP_INFO(logger,"[MULTI ROBOT COORDINATOR] Got request to register namespace %s", request->namespace_canditate.c_str());
	// --- Validate namespace ---
	int validation_result;
	size_t invalid_index;

	rmw_ret_t rmw_ret =
		rmw_validate_namespace(request->namespace_canditate.c_str(), &validation_result, &invalid_index);

	RCLCPP_DEBUG(logger, "[MULTI ROBOT COORDINATOR] Validate namespace %s: Validate if validation itself failed", request->namespace_canditate.c_str());
	// Check if validation itself failed
	if (rmw_ret != RMW_RET_OK) {
		if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
			rclcpp::exceptions::throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate subnode namespace");
		}
		rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, "failed to validate subnode namespace");
	}
	RCLCPP_DEBUG(logger, "[MULTI ROBOT COORDINATOR] Validate namespace %s: Validate if namespace is legal", request->namespace_canditate.c_str());
	// Check result from validatiom
	if (validation_result != RMW_NAMESPACE_VALID) {
		RCLCPP_WARN(logger, "[MULTI ROBOT COORDINATOR] Namespace %s isn't legal: Validation response %d", request->namespace_canditate.c_str(), validation_result);
		response->statuscode = sopias4_msgs::srv::Register::Response::ILLEGAL_NAMESPACE_ERROR;
		return;
	}
	RCLCPP_DEBUG(logger, "[MULTI ROBOT COORDINATOR] Validate namespace %s: Validate if namespace isn't already registered", request->namespace_canditate.c_str());
	// --- Check if namespace is already registered --
	if (std::find(registered_namespaces.begin(), registered_namespaces.end(), request->namespace_canditate) != registered_namespaces.end()) {
		response->statuscode = sopias4_msgs::srv::Register::Response::COLLISION_ERROR;
		return;
	}
	RCLCPP_DEBUG(logger, "[MULTI ROBOT COORDINATOR] Validated namespace %s", request->namespace_canditate.c_str());

	// --- Register namespace ---
	registered_namespaces.push_back(request->namespace_canditate);
	response->statuscode = sopias4_msgs::srv::Register::Response::SUCCESS;

	// Use a callback factory to pass a second argument to callback function (known bug in ros2, so this workaround is needed)
	std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)> callback_fcn = std::bind(&MultiRobotCoordinator::set_pose_callback, this, std::placeholders::_1, request->namespace_canditate);

	this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(request->namespace_canditate+ std::string("/amcl_pose"), 10, callback_fcn);
	this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(request->namespace_canditate+ std::string("/pose"), 10, callback_fcn);

	RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Successfully registered namespace %s", request->namespace_canditate.c_str());
}

void unregister_callback(const sopias4_msgs::srv::Unregister::Request::SharedPtr request, sopias4_msgs::srv::Unregister::Response::SharedPtr response){
    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Got request to unregister namespace %s", request ->name_space.c_str());
	response->statuscode = sopias4_msgs::srv::Unregister::Response::UNKNOWN_ERROR;
	// --- Delete namespace --
	for(auto element = registered_namespaces.begin(); element != registered_namespaces.end(); ++element) {
		response->statuscode = sopias4_msgs::srv::Unregister::Response::NS_NOT_FOUND;
		if( *element == request->name_space) {
			registered_namespaces.erase(element);
			response->statuscode = sopias4_msgs::srv::Unregister::Response::SUCCESS;
			break;
		}
	}
    RCLCPP_DEBUG(logger,"[MULTI ROBOT COORDINATOR] Deleted namespace %s from the internal list", request ->name_space.c_str());

	// --- Delete state of robot ---
	for(auto element = robot_states.begin(); element != robot_states.end(); ++element) {
		if( element->name_space == request->name_space) {
			robot_states.erase(element);
			response->statuscode = sopias4_msgs::srv::Unregister::Response::SUCCESS;
			break;
		}
	}
    RCLCPP_DEBUG(logger, "[MULTI ROBOT COORDINATOR] Deleted state of robot with namespace %s", request -> name_space.c_str());

    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Successfully unregistered namespace %s", request -> name_space.c_str());
}

void set_robot_path_callback(const sopias4_msgs::srv::SetRobotPath::Request::SharedPtr request, sopias4_msgs::srv::SetRobotPath::Response::SharedPtr response){
    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Got request to set path for robot with namespace %s", request ->name_space.c_str());
	for(auto element = robot_states.begin(); element != robot_states.end(); ++element) {
		response->statuscode = sopias4_msgs::srv::SetRobotPath::Response::ROBOT_NOT_FOUND;
		if( element->name_space == request->name_space) {
			element->nav_path = request->path;
			response->statuscode = sopias4_msgs::srv::SetRobotPath::Response::SUCCESS;
			break;
		}
	}

    RCLCPP_INFO(logger, "[MULTI ROBOT COORDINATOR] Successfully set path for robot with namespace %s", request ->name_space.c_str());
	return;
}

void set_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose, std::string name_space){
	for(auto element = robot_states.begin(); element != robot_states.end(); ++element) {
		if(element->name_space == name_space) {
			element->pose= *pose;
			return;
		}
	}
}
};

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	// Create node
	auto node = std::make_shared<MultiRobotCoordinator>("multi_robot_coordinator");

	// Run node
	rclcpp::spin(node);

	// Shutdown node
	rclcpp::shutdown();

	return 0;
}