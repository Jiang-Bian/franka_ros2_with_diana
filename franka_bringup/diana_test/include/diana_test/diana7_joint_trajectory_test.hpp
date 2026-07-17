#pragma once

#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class Diana7TrajectoryValidator : public rclcpp::Node {
 public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit Diana7TrajectoryValidator();

  //----------------------------------------
  // Initialization
  //----------------------------------------

  bool initialize();
  bool waitForActionServer(std::chrono::seconds timeout = std::chrono::seconds(5));
  bool waitForJointState(std::chrono::seconds timeout = std::chrono::seconds(5));

  //----------------------------------------
  // Joint State
  //----------------------------------------

  const sensor_msgs::msg::JointState& currentJointState() const;
  const std::vector<double>& currentVelocity() const;

  std::vector<double> target_position_;
  std::vector<double> current_position_;

  const std::vector<double>& targetPosition() const;
  const std::vector<double>& currentPosition() const;

  std::vector<double> computePositionError(const std::vector<double>& target) const;
  bool checkTargetPosition(const std::vector<double>& target) const;

  void printPosition() const;

  //----------------------------------------
  // Trajectory
  //----------------------------------------

  FollowJointTrajectory::Goal buildGoal();

  bool sendGoal(const FollowJointTrajectory::Goal& goal);

  //----------------------------------------
  // Result
  //----------------------------------------

  bool waitForResult(std::chrono::seconds timeout = std::chrono::seconds(10));
  bool succeeded() const;
  double executionTime() const;

 private:
  //----------------------------------------
  // ROS Callback
  //----------------------------------------

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle);
  void resultCallback(const GoalHandle::WrappedResult& result);

  //----------------------------------------
  // ROS Interface
  //----------------------------------------

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  //----------------------------------------
  // Cached JointState
  //----------------------------------------

  sensor_msgs::msg::JointState joint_state_;
  mutable std::mutex joint_state_mutex_;

  bool joint_state_received_{false};

  //----------------------------------------
  // Goal State
  //----------------------------------------

  GoalHandle::SharedPtr goal_handle_;

  bool goal_accepted_{false};
  bool result_received_{false};
  bool success_{false};

  rclcpp_action::ResultCode result_code_;

  //----------------------------------------
  // Timing
  //----------------------------------------

  rclcpp::Time start_time_;
  rclcpp::Time finish_time_;

  //----------------------------------------
  // Parameters
  //----------------------------------------
  int joint_index_;
  std::vector<std::string> joint_names_;
  double move_delta_;
  double move_duration_;
  bool wait_before_execute_;

  //----------------------------------------
  // joint limits
  //----------------------------------------
  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;
};