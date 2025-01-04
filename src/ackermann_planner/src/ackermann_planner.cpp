#include "ackermann_planner/ackermann_planner.hpp"

#include <cmath>
//#include <math.h>
//#include <stdint.h>
#include <string.h>
#include <iostream>
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
	this->wheelBase = 90.0 * 0.0075; //0.75(ratio) % 100(cm to m)
	this->maxSteeringAngle = 30.0;

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

	poseArrayPub = node->create_publisher<geometry_msgs::msg::PoseArray>(
			"/pose_array_topic", 10);
	validPosePub = node->create_publisher<geometry_msgs::msg::PoseArray>(
			"/valid_pose_topic", 10);
	invalidPosePub = node->create_publisher<geometry_msgs::msg::PoseArray>(
			"/invalid_pose_topic", 10);

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

	RCLCPP_INFO(
		logger_, "CostMap resolution: %f", costmap_->getResolution());

	response->success = true;
	response->message = "All good";
}

nav_msgs::msg::Path AckermannPlanner::createPlan(
	const geometry_msgs::msg::PoseStamped & start,
	const geometry_msgs::msg::PoseStamped & goal,
	std::function<bool()> cancel_checker)
{
	RCLCPP_INFO(
		logger_, "AckermannPlanner:: CreatePlan Method called");
	nav_msgs::msg::Path path;

	unsigned int mx_start, my_start;
	unsigned int mx, my;
	//std::stringstream strMap;

	if (!costmap_->worldToMap(
			start.pose.position.x,
			start.pose.position.y,
			mx_start,
			my_start))
	{
		throw nav2_core::StartOutsideMapBounds(
			"Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
			std::to_string(start.pose.position.y) + ") was outside bounds");
	}

	geometry_msgs::msg::PoseArray pose_array;
	pose_array.header.stamp = clock_->now();
	pose_array.header.frame_id = "map";

	geometry_msgs::msg::PoseArray invalidPoses;
	invalidPoses.header.stamp = clock_->now();
	invalidPoses.header.frame_id = "map";

	geometry_msgs::msg::PoseArray validPoses;
	validPoses.header.stamp = clock_->now();
	validPoses.header.frame_id = "map";


	double qx = start.pose.orientation.x;
	double qy = start.pose.orientation.y;
	double qz = start.pose.orientation.z;
	double qw = start.pose.orientation.w;

	double yaw = std::atan2(
			2.0 * (qw * qz + qx * qy),
			1.0 - 2.0 * (qy * qy + qz * qz)
	);

	//This is all temp
	int steps = 20;
	int stepInc = (this->maxSteeringAngle * 2)/steps;
	for (int i=0; i<=steps ; i++)
	{
		double ry, rx, deltaTheta;

		for (double j=0.0; j<18; j+=0.5){
		double testAngle = double(-this->maxSteeringAngle) + (i * stepInc);
		if(testAngle == 0){
			ry = 0;
			rx = j;
			deltaTheta = 0;
		}else{
			double radSteering = testAngle * M_PI / 180.0;
			double icrY = this->wheelBase / std::tan(radSteering);

			deltaTheta = j/icrY;
			if (std::abs(deltaTheta) > 2 * M_PI)
				break; //We have done full circle
			ry = (icrY * std::cos(deltaTheta)) - icrY;
			rx = icrY * std::sin(deltaTheta);
		}
		RCLCPP_INFO(
			logger_, "AckermannPlanner:: testAngle: %lf, delta: %lf",
				testAngle, deltaTheta);

		geometry_msgs::msg::Pose pose;
		double rads = yaw;
		pose.position.x = start.pose.position.x +
			(rx * std::cos(rads)) + (ry * std::sin(rads));
		pose.position.y = start.pose.position.y +
			(rx * std::sin(rads)) - (ry * std::cos(rads));
		pose.position.z = 0.0;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = std::sin((yaw+deltaTheta) / 2.0);
		pose.orientation.w = std::cos((yaw+deltaTheta)/ 2.0);

		pose_array.poses.push_back(pose);

		if (!costmap_->worldToMap(
				pose.position.x,
				pose.position.y,
				mx,
				my))
		{
			invalidPoses.poses.push_back(pose);
			break;
		}else{
			if(costmap_->getCost(mx, my) < 252){
				validPoses.poses.push_back(pose);
			}else{
				invalidPoses.poses.push_back(pose);
				break;
			}
		}

		}

	}

	geometry_msgs::msg::Pose pose;
	pose.position.x = start.pose.position.x;
	pose.position.y = start.pose.position.y;
	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = std::sin(yaw / 2.0);
	pose.orientation.w = std::cos(yaw / 2.0);
	pose_array.poses.push_back(pose);

	poseArrayPub->publish(pose_array);
	validPosePub->publish(validPoses);
	invalidPosePub->publish(invalidPoses);

	//for (unsigned int i=mx_start-5;i<mx_start+10;i++)
	//{
	//	for (unsigned int j=my_start-5;j<my_start+10;j++)
	//	{
	//		strMap << "C:" << static_cast<unsigned int>(costmap_->getCost(i, j));
	//		//printf("%03d ",costmap_->getCost(i, j));
	//	}
	//	strMap << std::	endl;
	//	//printf("\n");

	//}
	//RCLCPP_INFO(
	//	logger_, "AckermannPlanner:: Map\n %s", strMap.str().c_str());


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
