#ifndef GOAL_TO_ACTION__ACTION_PUBLISHER_HPP_
#define GOAL_TO_ACTION__ACTION_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"


namespace goal_to_action
{

class ActionPublisher: public nav2_util::LifecycleNode
{
public:
	using ActionT = nav2_msgs::action::ComputePathToPose;

	explicit ActionPublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());

	~ActionPublisher();

	/**
	 * @brief A subscription and callback to handle the topic-based goal published
	 * from rviz
	 * @param pose Pose received via atopic
	 */
	void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

	/**
	 * @brief Get action name for this navigator
	 * @return string Name of action server
	 */
	std::string getName() {return std::string("compute_path_to_pose");}

protected:
	/**
	 * @brief Configures member variables
	 *
	 * Initializes action server for "NavigationToPose"; subscription to
	 * "goal_sub"; and builds behavior tree from xml file.
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
	/**
	 * @brief Activates action server
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
	/**
	 * @brief Deactivates action server
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
	/**
	 * @brief Resets member variables
	 * @param state Reference to LifeCycle node state
	  * @return SUCCESS or FAILURE
	 */
	nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
	/**
	 * @brief Called when in shutdown state
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
	rclcpp_action::Client<ActionT>::SharedPtr self_client_;
};


} // namespace goal_to_action

#endif // GOAL_TO_ACTION__ACTION_PUBLISHER_HPP_
