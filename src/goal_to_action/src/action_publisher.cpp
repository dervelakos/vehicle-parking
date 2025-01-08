
#include "goal_to_action/action_publisher.hpp"

namespace goal_to_action
{


ActionPublisher::ActionPublisher(rclcpp::NodeOptions options)
: nav2_util::LifecycleNode("action_publisher", "",
	options.automatically_declare_parameters_from_overrides(true))
{

}

ActionPublisher::~ActionPublisher()
{
}

nav2_util::CallbackReturn
ActionPublisher::on_configure(const rclcpp_lifecycle::State & state)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	self_client_ = rclcpp_action::create_client<ActionT>(this, getName());

	goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
		"goal_pose",
		rclcpp::SystemDefaultsQoS(),
		std::bind(&ActionPublisher::onGoalPoseReceived, this, std::placeholders::_1));

	return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ActionPublisher::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ActionPublisher::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ActionPublisher::on_cleanup(const rclcpp_lifecycle::State & state)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");

	goal_sub_.reset();
	self_client_.reset();
	return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ActionPublisher::on_shutdown(const rclcpp_lifecycle::State & state)
{
	RCLCPP_INFO(get_logger(), "Shutting down");
	return nav2_util::CallbackReturn::SUCCESS;
}

void
ActionPublisher::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
	RCLCPP_INFO(get_logger(), "New goal");
	ActionT::Goal goal;
	goal.goal = *pose;
	self_client_->async_send_goal(goal);
}

} // namespace goal_to_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(goal_to_action::ActionPublisher)
