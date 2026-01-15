#include <string>
#include <memory>

#include "nav2_bt_navigator/navigators/navigate_with_path.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace nav2_bt_navigator
{

bool NavigateWithPathNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  auto node = parent_node.lock();
  start_time_ = rclcpp::Time(0);

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  odom_smoother_ = odom_smoother;

  return true;
}

std::string NavigateWithPathNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  std::string bt_xml;

  if (!node->has_parameter("default_nav_with_path_bt_xml")) {
    std::string pkg_share =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");

    node->declare_parameter<std::string>(
      "default_nav_with_path_bt_xml",
      pkg_share + "/behavior_trees/navigate_with_path.xml");
  }

  node->get_parameter("default_nav_with_path_bt_xml", bt_xml);
  return bt_xml;
}

bool NavigateWithPathNavigator::cleanup()
{
  return true;
}

bool NavigateWithPathNavigator::goalReceived(
  ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml)) {
    RCLCPP_ERROR(
      logger_,
      "Failed to load BT: %s",
      bt_xml.c_str());
    return false;
  }

  return initializePath(goal);
}

bool NavigateWithPathNavigator::initializePath(
  ActionT::Goal::ConstSharedPtr goal)
{
  if (goal->path.poses.empty()) {
    RCLCPP_ERROR(logger_, "Received empty path.");
    return false;
  }

  start_time_ = clock_->now();

  auto blackboard = bt_action_server_->getBlackboard();

  blackboard->set("number_recoveries", 0);  // optional
  blackboard->set(path_blackboard_id_, goal->path);

  RCLCPP_INFO(
    logger_,
    "NavigateWithPath: received path with %zu poses",
    goal->path.poses.size());

  return true;
}

void NavigateWithPathNavigator::onLoop()
{
  auto feedback = std::make_shared<ActionT::Feedback>();

  auto blackboard = bt_action_server_->getBlackboard();
  nav_msgs::msg::Path path;

  if (blackboard->get(path_blackboard_id_, path) && !path.poses.empty()) {
    feedback->progress = 1.0f;  // 你之后可以改成基于索引 / 距离
  } else {
    feedback->progress = 0.0f;
  }

  bt_action_server_->publishFeedback(feedback);
}

void NavigateWithPathNavigator::onPreempt(
  ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "NavigateWithPath preempted");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
      goal->behavior_tree.empty())
  {
    if (!initializePath(bt_action_server_->acceptPendingGoal())) {
      bt_action_server_->terminatePendingGoal();
    }
  } else {
    bt_action_server_->terminatePendingGoal();
  }
}

void NavigateWithPathNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  result->error_code = 0;
  result->error_msg = "";

  RCLCPP_INFO(
    logger_,
    "NavigateWithPath finished with status %d",
    static_cast<int>(final_bt_status));
}

}  // namespace nav2_bt_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_bt_navigator::NavigateWithPathNavigator,
  nav2_core::NavigatorBase)
