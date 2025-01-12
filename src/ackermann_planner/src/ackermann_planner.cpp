#include "ackermann_planner/ackermann_planner.hpp"

#include <cmath>
//#include <math.h>
//#include <stdint.h>
#include <string.h>
#include <iostream>
#include <stack>
//#include <stdio.h>
//#include <functional>
//#include <chrono>
//#include <cstdlib>
//#include <memory>
#include "geometry_msgs/msg/quaternion.hpp"

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

double poseToYaw(geometry_msgs::msg::Pose p, bool degrees = true) {
	/**
	* Converts the qw and qz components of a quaternion (representing a rotation
	* around the Z-axis) to yaw angle.
	*
	* Args:
	*   qw: The real component of the quaternion.
	*   qz: The z component of the imaginary part of the quaternion.
	*   degrees: If true, return the yaw in degrees. Otherwise, return in radians.
	*            Defaults to false (radians).
	*
	* Returns:
	*   The yaw angle of the 2D plane.
	*/
	double qx = p.orientation.x;
	double qy = p.orientation.y;
	double qz = p.orientation.z;
	double qw = p.orientation.w;

	double yaw_radians = std::atan2(
			2.0 * (qw * qz + qx * qy),
			1.0 - 2.0 * (qy * qy + qz * qz)
	);

	if (degrees) {
		return yaw_radians * 180.0 / M_PI;
	} else {
		return yaw_radians;
	}
}

AckermannPlanner::AckermannPlanner()
: tf_(nullptr), costmap_(nullptr)
{
	this->wheelBase = 90.0 * 0.01; //0.75(ratio) % 100(cm to m)
	this->maxSteeringAngle = 30.0;

	openList = new PlanNodeList();
	deleteList = new PlanNodeList();
	stepFlag = true;

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

	closedPoints = new ClosedPointsPlane(costmap_->getSizeInCellsX(),
										 costmap_->getSizeInCellsY(),
										 10); //10 degrees rotation tolerance

	//node_ = parent;
	auto node = parent.lock();
	clock_ = node->get_clock();
	logger_ = node->get_logger();

	service = node->create_service<std_srvs::srv::Trigger>("step",
		std::bind(&AckermannPlanner::step, this,
		std::placeholders::_1,
		std::placeholders::_2));

	validPosePub = node->create_publisher<geometry_msgs::msg::PoseArray>(
			"/valid_pose_topic", 10);
	invalidPosePub = node->create_publisher<geometry_msgs::msg::PoseArray>(
			"/invalid_pose_topic", 10);
	pathPub = node->create_publisher<geometry_msgs::msg::PoseArray>(
			"/path_pose_topic", 10);

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

	RCLCPP_INFO(
		logger_, "CostMap cells: %u, %u", costmap_->getSizeInCellsX(),
		costmap_->getSizeInCellsY());

	stepFlag = true;

	response->success = true;
	response->message = "All good";
}

geometry_msgs::msg::Pose
generatePose(const geometry_msgs::msg::Pose initialPose,
			 double angle,
			 double distance,
			 double wheelBase,
			 double *deltaTheta)
{
	double ry, rx;

	if(angle == 0){
		ry = 0;
		rx = distance;
		*deltaTheta = 0.0;
	}else{
		double radSteering = angle * M_PI / 180.0;
		double icrY = wheelBase / std::tan(radSteering);

		*deltaTheta = distance/icrY;
		ry = (icrY * std::cos(*deltaTheta)) - icrY;
		rx = icrY * std::sin(*deltaTheta);
	}

	geometry_msgs::msg::Pose pose;
	double yaw = poseToYaw(initialPose, false);
	pose.position.x = initialPose.position.x +
		(rx * std::cos(yaw)) - (ry * std::sin(yaw));
	pose.position.y = initialPose.position.y +
		(rx * std::sin(yaw)) + (ry * std::cos(yaw));

	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = std::sin((yaw-*deltaTheta) / 2.0);
	pose.orientation.w = std::cos((yaw-*deltaTheta)/ 2.0);

	return pose;
}

bool
poseInToleranceRange(const geometry_msgs::msg::Pose goal,
					const geometry_msgs::msg::Pose newPose,
					double tolerance)
{
	if (std::abs(goal.position.x - newPose.position.x) > tolerance)
		return false;
	if (std::abs(goal.position.y - newPose.position.y) > tolerance)
		return false;
	if (std::abs(goal.position.z - newPose.position.z) > tolerance)
		return false;

	if (std::abs(goal.orientation.x - newPose.orientation.x) > tolerance)
		return false;
	if (std::abs(goal.orientation.y - newPose.orientation.y) > tolerance)
		return false;
	if (std::abs(goal.orientation.z - newPose.orientation.z) > tolerance)
		return false;
	if (std::abs(goal.orientation.w - newPose.orientation.w) > tolerance)
		return false;

	return true;
}

PlanNode*
AckermannPlanner::createPlanNode(PlanNode *parent,
			   const geometry_msgs::msg::Pose goal,
			   const geometry_msgs::msg::Pose prev,
			   geometry_msgs::msg::Pose pose,
			   double angle,
			   double distance)
{
	int iteration = 0;
	if (parent)
		iteration = parent->getIteration() + 1;

	double cost = 0;
	if(parent)
		cost += parent->getCost();
	cost += iteration * 10000;
	cost += abs(goal.position.x - pose.position.x);
	cost += abs(goal.position.y - pose.position.y);
	cost += abs(poseToYaw(goal) - poseToYaw(pose));

	cost += (9 - abs(pose.position.x - prev.position.x)) * 1000;
	cost += (9 - abs(pose.position.y - prev.position.y)) * 1000;

	PlanNode *node = new PlanNode(pose, iteration, cost, angle, distance, parent);

	if (node == NULL)
		RCLCPP_INFO(
			logger_, "AckermannPlanner:: Couldn't allocate");

	return node;
}

void AckermannPlanner::createArcPath(geometry_msgs::msg::Pose initial,
				   PlanNode* node,
				   nav_msgs::msg::Path *path)
{
	double theta;
	const int direction = (node->getDistance() > 0) - (node->getDistance() < 0);
	const double stepLength = 0.5 * direction;
	const double steps = std::abs(node->getDistance()) / 0.5;

	for (int j=1; j<=steps; j++){
		geometry_msgs::msg::PoseStamped p;
		p.pose = generatePose(initial, node->getAngle(), stepLength * j,
				this->wheelBase, &theta);

		path->poses.push_back(p);

	}
}

PlanNode* AckermannPlanner::registerPose( geometry_msgs::msg::Pose pose,
		geometry_msgs::msg::Pose goal,
		geometry_msgs::msg::Pose cur,
		PlanNode *parent,
		double angle,
		double distance,
		bool *found)
{
	unsigned int mx, my;

	*found = false;

	if (!costmap_->worldToMap(
			pose.position.x,
			pose.position.y,
			mx,
			my))
	{
		return NULL;
	}
	if(costmap_->getCost(mx, my) >= 252){
		return NULL;
	}

	if(closedPoints->isClosed(mx, my, poseToYaw(pose))){
		return NULL;
	}
	closedPoints->closePoint(mx, my, poseToYaw(pose));

	PlanNode *tmpNode = createPlanNode(parent,
									goal,
									cur,
									pose,
									angle,
									distance);
	openList->insertNode(tmpNode);

	if (poseInToleranceRange(goal, pose, 0.4)){
		RCLCPP_INFO(
			logger_, "AckermannPlanner:: Found path!\n");
		*found = true;
	}

	return tmpNode;

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

	geometry_msgs::msg::PoseArray invalidPoses;
	invalidPoses.header.stamp = clock_->now();
	invalidPoses.header.frame_id = "map";

	geometry_msgs::msg::PoseArray validPoses;
	validPoses.header.stamp = clock_->now();
	validPoses.header.frame_id = "map";

	geometry_msgs::msg::PoseArray pathPoses;
	pathPoses.header.stamp = clock_->now();
	pathPoses.header.frame_id = "map";


	//This is all temp
	int steps = 20;
	int stepInc = (this->maxSteeringAngle * 2)/steps;
	PlanNode *parent = NULL, *found = NULL;
	geometry_msgs::msg::Pose cur = start.pose;
	geometry_msgs::msg::Pose newPose;

	bool pathFound = false;
	int lastGen = 0;

	if (costmap_->getSizeInCellsX() != closedPoints->getCellsX() ||
			costmap_->getSizeInCellsY() != closedPoints->getCellsY())
	{
		bool res = closedPoints->resize(costmap_->getSizeInCellsX(),
							 costmap_->getSizeInCellsY());
		RCLCPP_INFO(
			logger_, "AckermannPlanner:: resize done(%d)", res);
	}

	//openList->printList();
	//deleteList->printList();
	//RCLCPP_INFO(
	//	logger_, "AckermannPlanner:: OpenList\n%s", openList->printList().c_str());
	//RCLCPP_INFO(
	//	logger_, "AckermannPlanner:: DeleteList\n%s", deleteList->printList().c_str());
	closedPoints->nextEpoch();

	while(!pathFound){

		if(cancel_checker())
			break;

		if(!stepFlag){
			usleep(1000);
			continue;
		}

		//stepFlag = false;

		for (int i=0; i<=steps ; i++)
		{
			double testAngle = double(-this->maxSteeringAngle) + (i*stepInc);
			bool forwardValid = true;
			bool reverseValid = true;
			bool res = false;

			if (parent && parent->getAngle() == testAngle)
				continue;

			for (double j=0.5; j<9; j+=0.5){
				double deltaThetaForward = 0;
				double deltaThetaReverse = 0;
				PlanNode *newNode;

				if (forwardValid){
					newPose = generatePose(cur, testAngle, j,
									this->wheelBase, &deltaThetaForward);
					newNode = registerPose(newPose, goal.pose, cur, parent,
										   testAngle, j, &res);
					if (newNode){
						validPoses.poses.push_back(newPose);
						if (res){
							pathFound = true;
							found = newNode;
						}
					}else{
						forwardValid = false;
						invalidPoses.poses.push_back(newPose);
					}
				}

				if (reverseValid){
					newPose = generatePose(cur, testAngle, -j,
									this->wheelBase, &deltaThetaReverse);
					newNode = registerPose(newPose, goal.pose, cur, parent,
										   testAngle, -j, &res);
					if (newNode){
						validPoses.poses.push_back(newPose);
						if (res){
							pathFound = true;
							found = newNode;
						}
					}else{
						reverseValid = false;
						invalidPoses.poses.push_back(newPose);
					}
				}

				if (pathFound)
					break;

				if (std::abs(deltaThetaForward) + std::abs(deltaThetaReverse) >
							2 * M_PI)
					break;

				if (!forwardValid && !reverseValid)
					break;
			}

			validPosePub->publish(validPoses);
			invalidPosePub->publish(invalidPoses);

			validPoses.poses.clear();
			invalidPoses.poses.clear();
		}

		parent = openList->removeLeastCost();
		if (parent)
			deleteList->insertNode(parent);

		if (parent == NULL)
			break;
		cur = parent->getPose();

		if (parent->getIteration() != lastGen){
			lastGen = parent->getIteration();
			RCLCPP_INFO(
				logger_, "AckermannPlanner:: New iteration: %d!\n", lastGen);
			//validPosePub->publish(validPoses);
		}

	}


	//openList->printList();

	validPosePub->publish(validPoses);
	invalidPosePub->publish(invalidPoses);


	if (pathFound) {
		PlanNode *unravel = found;
		std::stack<geometry_msgs::msg::Pose> poseList;
		std::stack<PlanNode*> tmpList;
		while (unravel != NULL) {
			tmpList.push(unravel);
			poseList.push(unravel->getPose());
			printf("Unravel:%d\n", unravel->getIteration());
			unravel = unravel->getParent();
		}

		// clear the plan, just in case
		path.poses.clear();

		path.header.stamp = clock_->now();
		path.header.frame_id = global_frame_;

		geometry_msgs::msg::PoseStamped origin;
		origin.pose = start.pose;
		path.poses.push_back(origin);

		geometry_msgs::msg::Pose prev = origin.pose;
		while (!tmpList.empty()) {
			PlanNode* node;
			geometry_msgs::msg::PoseStamped p;
			node = tmpList.top();
			p.pose = poseList.top();

			createArcPath(prev, node, &path);
			//p.pose = node->getPose();

			//path.poses.push_back(p);
			pathPoses.poses.push_back(p.pose);

			prev = node->getPose();
			tmpList.pop();
			poseList.pop();

		}
		pathPub->publish(pathPoses);
	}

	//ClearLists
	openList->emptyList();
	deleteList->emptyList();
	printf("List Cleared\n");

	return path;

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
