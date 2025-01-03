#include "ackermann_planner/ackermann_planner.hpp"

//#include <math.h>
//#include <stdint.h>
//#include <string.h>
//#include <stdio.h>
//#include <functional>
//#include <chrono>
//#include <cstdlib>
//#include <memory>

//#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/logger.hpp"


//#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#define UNUSED(expr) do { (void)(expr); } while (0)

using namespace std::chrono_literals;

namespace ackermann_planner
{

AckermannPlanner::AckermannPlanner()
: tf_(nullptr), costmap_(nullptr)
{

}

AckermannPlanner::~AckermannPlanner()
{
	RCLCPP_INFO(
		logger_, "Destroying plugin %s of type AckermannPlanner",
		name_.c_str());
}

void
AckermannPlanner::configure(
	const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
	std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
	std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
	tf_ = tf;
	name_ = name;
	costmap_ = costmap_ros->getCostmap();
	global_frame_ = costmap_ros->getGlobalFrameID();

	//node_ = parent;
	auto node = parent.lock();
	clock_ = node->get_clock();
	logger_ = node->get_logger();

	service = node->create_service<std_srvs::srv::Trigger>("step",
		std::bind(&AckermannPlanner::step, this,
		std::placeholders::_1,
		std::placeholders::_2));

	RCLCPP_INFO(
		logger_, "Configuring plugin %s of type AckermannPlanner",
		name_.c_str());

}

void
AckermannPlanner::activate()
{
	RCLCPP_INFO(
		logger_, "Activating plugin %s of type AckermannPlanner",
		name_.c_str());
}

void
AckermannPlanner::deactivate()
{
	RCLCPP_INFO(
		logger_, "Deactivating plugin %s of type AckermannPlanner",
		name_.c_str());
}

void
AckermannPlanner::cleanup()
{
	RCLCPP_INFO(
		logger_, "Cleaning up plugin %s of type AckermannPlanner",
		name_.c_str());
}

void
AckermannPlanner::step(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	UNUSED(request);
	RCLCPP_INFO(
		logger_, "TriggerService: Service triggered! Doing something...");

	response->success = true;
	response->message = "All good";
}

nav_msgs::msg::Path AckermannPlanner::createPlan(
	const geometry_msgs::msg::PoseStamped & start,
	const geometry_msgs::msg::PoseStamped & goal,
	std::function<bool()> cancel_checker)
{
	nav_msgs::msg::Path path;

	if (!makePlan(start.pose, goal.pose, 0, cancel_checker, path)) {
		throw nav2_core::NoValidPathCouldBeFound(
		"Failed to create plan.");
	}

	return path;
}

bool
AckermannPlanner::makePlan(
	const geometry_msgs::msg::Pose & start,
	const geometry_msgs::msg::Pose & goal, double tolerance,
	std::function<bool()> cancel_checker,
	nav_msgs::msg::Path & plan)
{
	UNUSED(goal);
	UNUSED(cancel_checker);
	UNUSED(tolerance);

	// clear the plan, just in case
	plan.poses.clear();

	plan.header.stamp = clock_->now();
	plan.header.frame_id = global_frame_;

	double wx = start.position.x;
	double wy = start.position.y;

	geometry_msgs::msg::PoseStamped pose;
	pose.pose.position.x = 10.0 + wx;
	pose.pose.position.y = 10.0 + wy;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 1.0;
	plan.poses.push_back(pose);

	return !plan.poses.empty();
}

} // namespace ackermann_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ackermann_planner::AckermannPlanner, nav2_core::GlobalPlanner)
