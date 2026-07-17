#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

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

  void print(const std::string& title) const {
    std::cout << "\n";
    std::cout << "====================================================\n";

    std::cout << " " << title << "\n";

    std::cout << "====================================================\n";

    std::string current;

    int passed = 0;

    for (const auto& r : results_) {
      if (r.category != current) {
        current = r.category;
        std::cout << "\n[" << current << "]\n";
      }

      std::cout << "  " << (r.passed ? "[PASS] " : "[FAIL] ") << std::left << std::setw(35)
                << r.item;

      if (!r.message.empty()) {
        std::cout << " : " << r.message;
      }

      std::cout << std::endl;

      if (r.passed)
        ++passed;
    }

    std::cout << "\n----------------------------------------------------\n";

    std::cout << "Compatibility Score : " << passed << " / " << results_.size() << std::endl;

    std::cout << "Overall : " << (passed == static_cast<int>(results_.size()) ? "PASS" : "FAIL")
              << std::endl;

    std::cout << "====================================================\n";
  }

 private:
  std::vector<TestResult> results_;
};