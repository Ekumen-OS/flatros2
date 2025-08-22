// benchmark_pub_fixed_size.cpp
// Publishes fixed-size messages for benchmarking, similar to benchmark_pub.py
// Usage: ros2 run <package> benchmark_pub_fixed_size --sizes 10 100kb 1mb 2mb 4mb --rate <hz>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "benchmark_msgs/msg/stamped10_b.hpp"
#include "benchmark_msgs/msg/stamped100_kb.hpp"
#include "benchmark_msgs/msg/stamped1_mb.hpp"
#include "benchmark_msgs/msg/stamped2_mb.hpp"
#include "benchmark_msgs/msg/stamped4_mb.hpp"

using namespace std::chrono_literals;

struct SizeInfo {
  std::string name;
  size_t size_bytes;
};

const std::unordered_map<std::string, SizeInfo> size_map = {
  {"10", {"Stamped10B", 10}},
  {"100kb", {"Stamped100KB", 100 * 1024}},
  {"1mb", {"Stamped1MB", 1024 * 1024}},
  {"2mb", {"Stamped2MB", 2 * 1024 * 1024}},
  {"4mb", {"Stamped4MB", 4 * 1024 * 1024}},
};

class BenchmarkPubNode : public rclcpp::Node {
public:
  BenchmarkPubNode(const std::vector<std::string>& sizes, int period_ms)
    : Node("benchmark_pub_fixed_size"), sizes_(sizes), period_ms_(period_ms) {
    using namespace std::placeholders;
    for (const auto& size : sizes_) {
      if (size == "10") {
        pub10_ = this->create_publisher<benchmark_msgs::msg::Stamped10B>("benchmark/fixed/10", 10);
      } else if (size == "100kb") {
        pub100kb_ = this->create_publisher<benchmark_msgs::msg::Stamped100KB>("benchmark/fixed/100kb", 10);
      } else if (size == "1mb") {
        pub1mb_ = this->create_publisher<benchmark_msgs::msg::Stamped1MB>("benchmark/fixed/1mb", 10);
      } else if (size == "2mb") {
        pub2mb_ = this->create_publisher<benchmark_msgs::msg::Stamped2MB>("benchmark/fixed/2mb", 10);
      } else if (size == "4mb") {
        pub4mb_ = this->create_publisher<benchmark_msgs::msg::Stamped4MB>("benchmark/fixed/4mb", 10);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown size: %s", size.c_str());
      }
    }
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&BenchmarkPubNode::timer_callback, this));
  }

private:
  void timer_callback() {
    // No initialization, always use loaned messages (assume support)
    for (const auto& size : sizes_) {
      if (size == "10" && pub10_) {
        std::cout << "Publishing size: 10 bytes" << std::endl;
        auto loaned = pub10_->borrow_loaned_message();
        pub10_->publish(std::move(loaned));
      } else if (size == "100kb" && pub100kb_) {
        std::cout << "Publishing size: 100 KB" << std::endl;
        auto loaned = pub100kb_->borrow_loaned_message();
        pub100kb_->publish(std::move(loaned));
      } else if (size == "1mb" && pub1mb_) {
        std::cout << "Publishing size: 1 MB" << std::endl;
        auto loaned = pub1mb_->borrow_loaned_message();
        pub1mb_->publish(std::move(loaned));
      } else if (size == "2mb" && pub2mb_) {
        std::cout << "Publishing size: 2 MB" << std::endl;
        auto loaned = pub2mb_->borrow_loaned_message();
        pub2mb_->publish(std::move(loaned));
      } else if (size == "4mb" && pub4mb_) {
        std::cout << "Publishing size: 4 MB" << std::endl;
        auto loaned = pub4mb_->borrow_loaned_message();
        pub4mb_->publish(std::move(loaned));
      }
    }
  }

  std::vector<std::string> sizes_;
  int period_ms_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<benchmark_msgs::msg::Stamped10B>::SharedPtr pub10_;
  rclcpp::Publisher<benchmark_msgs::msg::Stamped100KB>::SharedPtr pub100kb_;
  rclcpp::Publisher<benchmark_msgs::msg::Stamped1MB>::SharedPtr pub1mb_;
  rclcpp::Publisher<benchmark_msgs::msg::Stamped2MB>::SharedPtr pub2mb_;
  rclcpp::Publisher<benchmark_msgs::msg::Stamped4MB>::SharedPtr pub4mb_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::vector<std::string> sizes;
  int period_ms = 100;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--sizes" && i + 1 < argc) {
      for (int j = i + 1; j < argc && argv[j][0] != '-'; ++j) {
        sizes.push_back(argv[j]);
        i = j;
      }
    } else if (arg == "--period" && i + 1 < argc) {
      period_ms = std::stoi(argv[++i]);
    }
  }
  if (sizes.empty()) {
    std::cerr << "Usage: benchmark_pub_fixed_size --sizes 10 100kb 1mb 2mb 4mb [--period <ms>]" << std::endl;
    return 1;
  }
  auto node = std::make_shared<BenchmarkPubNode>(sizes, period_ms);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
