#include "diana_test/diana7_joint_trajectory_test.hpp"
#include "diana_test/compatibility_report.hpp"

const std::vector<double> JOINT_LOWER_LIMIT = {-3.1241, -1.5708, -3.1241, 0.0000,
                                               -3.1241, -3.1241, -3.1241};

const std::vector<double> JOINT_UPPER_LIMIT = {3.1241, 1.5708, 3.1241, 3.0543,
                                               3.1241, 3.1241, 3.1241};

Diana7TrajectoryValidator::Diana7TrajectoryValidator()
    : Node("diana7_joint_trajectory_test"),
      lower_limit_(JOINT_LOWER_LIMIT),
      upper_limit_(JOINT_UPPER_LIMIT) {
  //----------------------------------------
  // Parameters
  //----------------------------------------

  declare_parameter<int>("joint_index", 0);
  declare_parameter<double>("delta", 0.20);
  declare_parameter<double>("duration", 3.0);
  declare_parameter<bool>("wait_before_execute", false);

  joint_index_ = get_parameter("joint_index").as_int();
  move_delta_ = get_parameter("delta").as_double();
  move_duration_ = get_parameter("duration").as_double();
  wait_before_execute_ = get_parameter("wait_before_execute").as_bool();

  //----------------------------------------
  // Joint Names
  //----------------------------------------
  joint_names_ = {"diana7_joint1", "diana7_joint2", "diana7_joint3", "diana7_joint4", "diana7_joint5", "diana7_joint6", "diana7_joint7"};

  //----------------------------------------
  // Subscriber
  //----------------------------------------
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&Diana7TrajectoryValidator::jointStateCallback, this, std::placeholders::_1));

  //----------------------------------------
  // Action Client
  //----------------------------------------
  action_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(this,
                                                          "/joint_trajectory_controller/"
                                                          "follow_joint_trajectory");
}

bool Diana7TrajectoryValidator::initialize() {
  RCLCPP_INFO(get_logger(), "Initializing Diana JointTrajectory Validator...");

  if (!waitForActionServer()) {
    return false;
  }

  if (!waitForJointState()) {
    return false;
  }

  return true;
}

bool Diana7TrajectoryValidator::waitForActionServer(std::chrono::seconds timeout) {
  RCLCPP_INFO(get_logger(), "Waiting for FollowJointTrajectory Action Server...");

  if (!action_client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(get_logger(), "FollowJointTrajectory Action Server not available.");

    return false;
  }

  RCLCPP_INFO(get_logger(), "Action Server connected.");

  return true;
}

bool Diana7TrajectoryValidator::waitForJointState(std::chrono::seconds timeout) {
  auto start = now();

  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);

      if (joint_state_received_) {
        RCLCPP_INFO(get_logger(), "Received JointState (%zu joints).",
                    joint_state_.position.size());

        return true;
      }
    }

    if ((now() - start) > rclcpp::Duration(timeout)) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for JointState.");

      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return false;
}

const std::vector<double>& Diana7TrajectoryValidator::currentPosition() const {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  return current_position_;
}
const std::vector<double>& Diana7TrajectoryValidator::targetPosition() const {
  return target_position_;
}

const std::vector<double>& Diana7TrajectoryValidator::currentVelocity() const {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  return joint_state_.velocity;
}

const sensor_msgs::msg::JointState& Diana7TrajectoryValidator::currentJointState() const {
  return joint_state_;
}

void Diana7TrajectoryValidator::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  joint_state_ = *msg;

  current_position_ = msg->position;

  joint_state_received_ = true;
}

Diana7TrajectoryValidator::FollowJointTrajectory::Goal Diana7TrajectoryValidator::buildGoal() {
  //----------------------------------------
  // Current Joint Position
  //----------------------------------------

  target_position_ = current_position_;

  if (target_position_.size() != joint_names_.size()) {
    throw std::runtime_error("Invalid JointState.");
  }

  //----------------------------------------
  // Build Target
  //----------------------------------------

  target_position_[joint_index_] += move_delta_;

  //----------------------------------------
  // Joint Limit Check
  //----------------------------------------

  if (!checkTargetPosition(target_position_)) {
    throw std::runtime_error("Target position exceeds joint limits.");
  }

  //----------------------------------------
  // Optional Preview
  //----------------------------------------

  printPosition();

  if (wait_before_execute_) {
    std::cout << "\nPress ENTER to execute...";
    std::cin.get();
  }

  //----------------------------------------
  // Build Trajectory
  //----------------------------------------

  trajectory_msgs::msg::JointTrajectory trajectory;

  trajectory.joint_names = joint_names_;

  //
  // Point 0 : Current
  //
  trajectory_msgs::msg::JointTrajectoryPoint start;

  start.positions = current_position_;
  start.time_from_start = rclcpp::Duration::from_seconds(0.0);

  trajectory.points.push_back(start);

  //
  // Point 1 : Target
  //
  trajectory_msgs::msg::JointTrajectoryPoint target;

  target.positions = target_position_;
  target.time_from_start = rclcpp::Duration::from_seconds(move_duration_);

  trajectory.points.push_back(target);

  //----------------------------------------
  // Build Goal
  //----------------------------------------

  FollowJointTrajectory::Goal goal;

  goal.trajectory = std::move(trajectory);

  return goal;
}

bool Diana7TrajectoryValidator::sendGoal(const FollowJointTrajectory::Goal& goal) {
  goal_accepted_ = false;
  result_received_ = false;
  success_ = false;

  start_time_ = std::chrono::steady_clock::now();

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;

  options.goal_response_callback =
      std::bind(&Diana7TrajectoryValidator::goalResponseCallback, this, std::placeholders::_1);

  options.result_callback =
      std::bind(&Diana7TrajectoryValidator::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal, options);

  return true;
}

void Diana7TrajectoryValidator::goalResponseCallback(GoalHandle::SharedPtr goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal rejected.");

    goal_accepted_ = false;

    return;
  }

  goal_handle_ = goal_handle;

  goal_accepted_ = true;

  RCLCPP_INFO(get_logger(), "Goal accepted.");
}

void Diana7TrajectoryValidator::resultCallback(const GoalHandle::WrappedResult& result) {
  finish_time_ = std::chrono::steady_clock::now();
  result_received_ = true;
  result_code_ = result.code;

  success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:

      RCLCPP_INFO(get_logger(), "Trajectory execution succeeded.");

      break;

    case rclcpp_action::ResultCode::ABORTED:

      RCLCPP_ERROR(get_logger(), "Trajectory aborted.");

      break;

    case rclcpp_action::ResultCode::CANCELED:

      RCLCPP_WARN(get_logger(), "Trajectory canceled.");

      break;

    default:

      RCLCPP_ERROR(get_logger(), "Unknown action result.");

      break;
  }
}

bool Diana7TrajectoryValidator::waitForResult(std::chrono::seconds timeout) {
  auto start = now();

  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());

    if (result_received_) {
      return success_;
    }

    if ((now() - start) > rclcpp::Duration(timeout)) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for trajectory result.");

      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return false;
}

bool Diana7TrajectoryValidator::succeeded() const {
  return success_;
}

double Diana7TrajectoryValidator::executionTime() const {
  return std::chrono::duration<double>(finish_time_ - start_time_).count();
}

std::vector<double> Diana7TrajectoryValidator::computePositionError(
    const std::vector<double>& target) const {
  auto current = currentPosition();

  std::vector<double> error(current.size());

  for (size_t i = 0; i < current.size(); ++i) {
    error[i] = std::abs(current[i] - target[i]);
  }

  return error;
}

bool Diana7TrajectoryValidator::checkTargetPosition(const std::vector<double>& target) const {
  for (size_t i = 0; i < target.size(); ++i) {
    if (target[i] < lower_limit_[i]) {
      RCLCPP_ERROR(get_logger(),
                   "Joint %zu below lower limit "
                   "(%.3f < %.3f)",
                   i + 1, target[i], lower_limit_[i]);

      return false;
    }

    if (target[i] > upper_limit_[i]) {
      RCLCPP_ERROR(get_logger(),
                   "Joint %zu above upper limit "
                   "(%.3f > %.3f)",
                   i + 1, target[i], upper_limit_[i]);

      return false;
    }
  }

  return true;
}

void Diana7TrajectoryValidator::printPosition() const {
  std::cout << "\nCurrent -> Target\n";

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    std::cout << joint_names_[i] << " : " << current_position_[i] << " -> " << target_position_[i]
              << std::endl;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Diana7TrajectoryValidator>();

  CompatibilityReport report;

  //--------------------------------------
  // Initialize
  //--------------------------------------
  if (!node->initialize()) {
    return EXIT_FAILURE;
  }

  //--------------------------------------
  // Build Goal
  //--------------------------------------
  auto goal = node->buildGoal();

  //--------------------------------------
  // Send Goal
  //--------------------------------------
  node->sendGoal(goal);

  //--------------------------------------
  // Wait Result
  //--------------------------------------
  bool success = node->waitForResult();

  report.add("Controller", "Trajectory Execution", success);

  //--------------------------------------
  // Position Error
  //--------------------------------------
  auto error = node->computePositionError(node->targetPosition());

  constexpr double tol = 0.01;

  for (size_t i = 0; i < error.size(); ++i) {
    report.add("Position", "Joint" + std::to_string(i + 1), error[i] < tol,
               std::to_string(error[i]));
  }

  report.add("Timing", "Execution Time", true, std::to_string(node->executionTime()) + " s");

  //--------------------------------------

  report.print("Diana7 JointTrajectory Test Report");

  rclcpp::shutdown();

  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}