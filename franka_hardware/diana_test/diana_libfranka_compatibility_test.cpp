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

using Clock = std::chrono::high_resolution_clock;

template <typename Func>
double measureExecutionTime(Func&& f) {
  auto t0 = Clock::now();

  f();

  auto t1 = Clock::now();

  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

struct TestResult {
  std::string category;
  std::string item;
  bool passed;
  std::string message;
};

class CompatibilityReport {
 public:
  void add(const std::string& category,
           const std::string& item,
           bool passed,
           const std::string& msg = "") {
    results_.push_back({category, item, passed, msg});
  }

  void print() const {
    std::cout << "\n";
    std::cout << "=========================================================\n";
    std::cout << "     Diana libfranka Compatibility Report\n";
    std::cout << "=========================================================\n\n";

    std::string current_category;

    int passed = 0;

    for (const auto& r : results_) {
      if (r.category != current_category) {
        current_category = r.category;
        std::cout << "\n[" << current_category << "]\n";
      }

      std::cout << "  " << (r.passed ? "[PASS] " : "[FAIL] ") << std::left << std::setw(32)
                << r.item;

      if (!r.message.empty())
        std::cout << " : " << r.message;

      std::cout << std::endl;

      if (r.passed)
        ++passed;
    }

    std::cout << "\n---------------------------------------------------------\n";

    std::cout << "Compatibility Score : " << passed << " / " << results_.size() << std::endl;

    std::cout << "Overall : " << (passed == static_cast<int>(results_.size()) ? "PASS" : "FAIL")
              << std::endl;

    std::cout << "=========================================================\n";
  }

 private:
  std::vector<TestResult> results_;
};

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
              << "    diana_libfranka_compatibility_test <robot_ip>\n";

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

    report.print();

  } catch (const franka::Exception& e) {
    report.add("Connection", "Robot Constructor", false, e.what());
    report.print();
    return EXIT_FAILURE;

  } catch (const std::exception& e) {
    report.add("General", "Exception", false, e.what());
    report.print();
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
