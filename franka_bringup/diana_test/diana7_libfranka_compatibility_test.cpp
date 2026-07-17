#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "diana_test/compatibility_report.hpp"

using Clock = std::chrono::high_resolution_clock;

template <typename Func>
double measureExecutionTime(Func&& f) {
  auto t0 = Clock::now();

  f();

  auto t1 = Clock::now();

  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}


template <typename T, std::size_t N>
bool isFiniteArray(const std::array<T, N>& values) {
  for (const auto& v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

template <typename T, std::size_t N>
void checkArray(CompatibilityReport& report,
                const std::string& category,
                const std::string& name,
                const std::array<T, N>& values) {
  bool ok = isFiniteArray(values);

  report.add(category, name, ok, ok ? ("size=" + std::to_string(N)) : "contains NaN/Inf");
}

bool checkJointVector(
    const std::array<double,7>& value)
{
    return isFiniteArray(value);
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage:\n"
              << "    diana7_libfranka_compatibility_test <robot_ip>\n";

    return EXIT_FAILURE;
  }

  CompatibilityReport report;

  const std::string ip = argv[1];

  try {
    //------------------------------------------------------
    // Connect
    //------------------------------------------------------

    auto t0 = Clock::now();

    franka::Robot robot(ip);

    auto t1 = Clock::now();

    report.add(
        "Connection", "Robot Constructor", true,
        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()) +
            " ms");

    //------------------------------------------------------
    // RobotState
    //------------------------------------------------------

    auto state = robot.readOnce();
    report.add("RobotState", "readOnce()", true);

    checkArray(report, "RobotState", "q", state.q);
    checkArray(report, "RobotState", "dq", state.dq);
    checkArray(report, "RobotState", "tau_J", state.tau_J);
    checkArray(report, "RobotState", "tau_J_d", state.tau_J_d);
    checkArray(report, "RobotState", "tau_ext_hat_filtered", state.tau_ext_hat_filtered);
    checkArray(report, "RobotState", "O_T_EE", state.O_T_EE);
    checkArray(report, "RobotState", "O_T_EE_d", state.O_T_EE_d);
    // checkArray(report, "RobotState", "O_dP_EE", state.O_dP_EE);
    checkArray(report, "RobotState", "elbow", state.elbow);
    // checkArray(report, "RobotState", "TCP Pose", state.O_T_EE);

    report.add("RobotState", "robot_mode", true,
               std::to_string(static_cast<int>(state.robot_mode)));

    report.add("RobotState", "robot_time", state.time.toSec() > 0.0,
               std::to_string(state.time.toSec()));

    //------------------------------------------------------
    // Model
    //------------------------------------------------------

    auto model = robot.loadModel();
    report.add("Model", "loadModel()", true);

    auto gravity = model.gravity(state);
    checkArray(report, "Model", "gravity()", gravity);

    auto coriolis = model.coriolis(state);
    checkArray(report, "Model", "coriolis()", coriolis);

    auto mass = model.mass(state);
    bool ok = isFiniteArray(mass);
    report.add("Model", "mass()", ok, "7x7");

    auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
    checkArray(report, "Model", "zeroJacobian()", jacobian);

    auto body = model.bodyJacobian(franka::Frame::kEndEffector, state);
    checkArray(report, "Model", "bodyJacobian()", body);

    auto pose = model.pose(franka::Frame::kEndEffector, state);
    checkArray(report, "Model", "pose()", pose);

    //------------------------------------------------------

    report.print("Diana7 libfranka Compatibility Test Report");

  } catch (const franka::Exception& e) {
    report.add("Connection", "Robot Constructor", false, e.what());
    report.print("Diana7 libfranka Compatibility Test Report");
    return EXIT_FAILURE;

  } catch (const std::exception& e) {
    report.add("General", "Exception", false, e.what());
    report.print("Diana7 libfranka Compatibility Test Report");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
