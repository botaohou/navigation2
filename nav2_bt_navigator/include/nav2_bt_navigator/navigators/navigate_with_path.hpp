#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_WITH_PATH_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_WITH_PATH_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/path.hpp"

#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_util/odometry_utils.hpp"

#include "nav2_msgs/action/navigate_with_path.hpp"

namespace nav2_bt_navigator
{

class NavigateWithPathNavigator
  : public nav2_core::BehaviorTreeNavigator<
      nav2_msgs::action::NavigateWithPath>
{
public:
  using ActionT = nav2_msgs::action::NavigateWithPath;

  NavigateWithPathNavigator()
  : BehaviorTreeNavigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  bool cleanup() override;

  std::string getName() override
  {
    return std::string("navigate_with_path");
  }

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

private:
  bool initializePath(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  std::string path_blackboard_id_;

  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_WITH_PATH_HPP_
