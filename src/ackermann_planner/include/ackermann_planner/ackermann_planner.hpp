#ifndef ACKERMANN_PLANNER__ACKERMANN_PLANNER_HPP_
#define ACKERMANN_PLANNER__ACKERMANN_PLANNER_HPP_

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
//#include "geometry_msgs/msg/point.hpp"
//#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_core/global_planner.hpp"

//#include "nav_msgs/msg/path.hpp"
//#include "nav2_util/robot_utils.hpp"
//#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
//#include "nav2_util/geometry_utils.hpp"

namespace ackermann_planner
{

class AckermannPlanner : public nav2_core::GlobalPlanner
{
public:
	/**
	* @brief constructor
	*/
	AckermannPlanner();

	/**
	* @brief destructor
	*/
	~AckermannPlanner();

	/**
	* @brief Configuring plugin
	* @param parent Lifecycle node pointer
	* @param name Name of plugin map
	* @param tf Shared ptr of TF2 buffer
	* @param costmap_ros Costmap2DROS object
	*/
	void configure(
		const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
		std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
		std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

	/**
	* @brief Cleanup lifecycle node
	*/
	void cleanup() override;

	/**
	* @brief Activate lifecycle node
	*/
	void activate() override;

	/**
	* @brief Deactivate lifecycle node
	*/
	void deactivate() override;

	/**
	* @brief Creating a plan from start and goal poses
	* @param start Start pose
	* @param goal Goal pose
	* @param cancel_checker Function to check if the task has been canceled
	* @return nav_msgs::Path of the generated path
	*/
	nav_msgs::msg::Path createPlan(
		const geometry_msgs::msg::PoseStamped & start,
		const geometry_msgs::msg::PoseStamped & goal,
		std::function<bool()> cancel_checker) override;

protected:
	/**
	* @brief Compute a plan given start and goal poses, provided a global world frame.
	* @param start Start pose
	* @param goal Goal pose
	* @param tolerance Relaxation constraint in x and y
	* @param cancel_checker Function to check if the task has been canceled
	* @param plan Path to be computed
	* @return true if can find the path
	*/
	bool makePlan(
		const geometry_msgs::msg::Pose & start,
		const geometry_msgs::msg::Pose & goal, double tolerance,
		std::function<bool()> cancel_checker,
		nav_msgs::msg::Path & plan);

	void step(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
			  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

	// TF buffer
	std::shared_ptr<tf2_ros::Buffer> tf_;

	// Clock
	rclcpp::Clock::SharedPtr clock_;

	// Logger
	rclcpp::Logger logger_{rclcpp::get_logger("AckermannPlanner")};

	// Global Costmap
	nav2_costmap_2d::Costmap2D * costmap_;

	// The global frame of the costmap
	std::string global_frame_, name_;

	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poseArrayPub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr validPosePub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr invalidPosePub;
	//Static definition for now, will import them as parameters later
	double wheelBase; //Front-back wheels seperation
	double maxSteeringAngle; //Maximum steering angle
};

} // namespace ackermann_planner

#endif  // ACKERMANN_PLANNER__ACKERMANN_PLANNER_HPP_
