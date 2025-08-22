#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "benchmark_msgs/msg/stamped10_b.hpp"
#include "benchmark_msgs/msg/stamped100_kb.hpp"
#include "benchmark_msgs/msg/stamped1_mb.hpp"
#include "benchmark_msgs/msg/stamped2_mb.hpp"
#include "benchmark_msgs/msg/stamped4_mb.hpp"

// Helper to extract timestamp from header
rclcpp::Time get_stamp(const std_msgs::msg::Header& header) {
  return rclcpp::Time(header.stamp.sec, header.stamp.nanosec, RCL_ROS_TIME);
}


// BenchmarkSubscriber node definition
class BenchmarkSubscriber : public rclcpp::Node {
public:
  BenchmarkSubscriber(bool log_to_file = true, bool debug = false)
    : Node("benchmark_subscriber_fixed_size", rclcpp::NodeOptions().start_parameter_services(false)),
      log_to_file_(log_to_file), debug_(debug)
  {
    if (log_to_file_) {
      logfile_.open("latency_log.csv");
      logfile_ << "[START] Program started\n";
      logfile_ << "size_bytes,latency_ms\n";
      logfile_.flush();
    }
    // Subscribe to all fixed-size topics
    sub10_ = this->create_subscription<benchmark_msgs::msg::Stamped10B>(
      "benchmark/fixed/10", 10,
      [this](const benchmark_msgs::msg::Stamped10B& msg) { handle_msg(msg); });
    sub100kb_ = this->create_subscription<benchmark_msgs::msg::Stamped100KB>(
      "benchmark/fixed/100kb", 10,
      [this](const benchmark_msgs::msg::Stamped100KB& msg) { handle_msg(msg); });
    sub1mb_ = this->create_subscription<benchmark_msgs::msg::Stamped1MB>(
      "benchmark/fixed/1mb", 10,
      [this](const benchmark_msgs::msg::Stamped1MB& msg) { handle_msg(msg); });
    sub2mb_ = this->create_subscription<benchmark_msgs::msg::Stamped2MB>(
      "benchmark/fixed/2mb", 10,
      [this](const benchmark_msgs::msg::Stamped2MB& msg) { handle_msg(msg); });
    sub4mb_ = this->create_subscription<benchmark_msgs::msg::Stamped4MB>(
      "benchmark/fixed/4mb", 10,
      [this](const benchmark_msgs::msg::Stamped4MB& msg) { handle_msg(msg); });
  }

  ~BenchmarkSubscriber() {
    if (log_to_file_ && logfile_.is_open()) {
      logfile_ << "[END] Program exiting\n";
      logfile_.flush();
      logfile_.close();
    }
  }

private:
  template<typename MsgT>
  void handle_msg(const MsgT& msg) {
    auto now = this->get_clock()->now();
    auto msg_time = get_stamp(msg.header);
    double latency_ms = (now - msg_time).seconds() * 1e3;
    size_t size = msg.data.size();
    if (log_to_file_ && logfile_.is_open()) {
      logfile_ << size << "," << latency_ms << "\n";
      logfile_.flush();
    }
    if (debug_) {
      std::cout << "[DEBUG] Received message of size: " << size
                << " bytes, latency: " << latency_ms << " ms" << std::endl;
    }
  }

  std::ofstream logfile_;
  rclcpp::Subscription<benchmark_msgs::msg::Stamped10B>::SharedPtr sub10_;
  rclcpp::Subscription<benchmark_msgs::msg::Stamped100KB>::SharedPtr sub100kb_;
  rclcpp::Subscription<benchmark_msgs::msg::Stamped1MB>::SharedPtr sub1mb_;
  rclcpp::Subscription<benchmark_msgs::msg::Stamped2MB>::SharedPtr sub2mb_;
  rclcpp::Subscription<benchmark_msgs::msg::Stamped4MB>::SharedPtr sub4mb_;
  bool log_to_file_;
  bool debug_;
};

void print_help() {
  const char* bold = "\033[1m";
  const char* green = "\033[32m";
  const char* yellow = "\033[33m";
  const char* cyan = "\033[36m";
  const char* reset = "\033[0m";
  std::cout << bold << green << "Usage:" << reset << " benchmark_sub_fixed_size [--duration <seconds>] [--log true|false] [--debug] [--help]\n";
  std::cout << yellow << "  --duration, -d <seconds>" << reset << "  Time to run before exiting (default: run until killed)\n";
  std::cout << yellow << "  --log true|false     " << reset << "  Enable or disable logging to file (default: true)\n";
  std::cout << yellow << "  --debug              " << reset << "  Print each received message's size and latency to console\n";
  std::cout << yellow << "  --help, -h           " << reset << "  Show this help message\n";
  std::cout << "\n";
  std::cout << cyan << "This program subscribes to fixed-size messages and logs latency data to 'latency_log.csv' (unless --log false).\n" << reset;
  std::cout << cyan << "You can analyze the latency data using the script 'analyze_latency.py'.\n" << reset;
  std::cout << bold << "Instructions to run the subscriber:" << reset << std::endl;
  std::cout << green << "  ./install/flatros2/lib/flatros2/benchmark_sub_fixed_size --duration 30" << reset << std::endl;
  std::cout << "This will start the subscriber and log latency data to 'latency_log.csv'." << std::endl;
  std::cout << "You can analyze the results with: " << yellow << "python3 /workspace/analyze_latency.py" << reset << std::endl;
  std::cout << std::endl;
  std::cout << bold << yellow << "Note:" << reset << " In another terminal, run the " << green << "publisher" << reset << " to send benchmark messages:" << std::endl;
  std::cout << "  " << cyan << "ros2 run flatros2 benchmark_pub_fixed_size --sizes 10 100kb 1mb 2mb 4mb --period 100" << reset << std::endl;
}

int main(int argc, char** argv) {
  std::cout << "\033[1;31m[WARNING]\033[0m Ctrl+C does not work to stop this program.\n";
  std::cout << "To kill all benchmark nodes and clean up shared memory, run:\n";
  std::cout << "  pkill -9 -f 'benchmark_pub|benchmark_sub|ros2'; sudo rm -f /dev/shm/iox2_*; sudo rm -rf /tmp/iceoryx2\n";
  std::cout << std::endl;
  int duration_sec = -1;
  bool log_to_file = true;
  bool debug = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_help();
      return 0;
    } else if ((arg == "--duration" || arg == "-d") && i + 1 < argc) {
      duration_sec = std::stoi(argv[++i]);
    } else if (arg == "--log" && i + 1 < argc) {
      std::string val = argv[++i];
      if (val == "false" || val == "0") log_to_file = false;
      else log_to_file = true;
    } else if (arg == "--debug") {
      debug = true;
    }
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BenchmarkSubscriber>(log_to_file, debug);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  if (duration_sec > 0) {
    std::atomic<bool> done{false};
    std::thread spin_thread([&]() {
      executor.spin();
      done = true;
    });
    auto start = std::chrono::steady_clock::now();
    while (!done) {
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= duration_sec) {
        std::cout << "Duration reached, shutting down." << std::endl;
        executor.cancel();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    spin_thread.join();
  } else {
    executor.spin();
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  // Print analysis instructions after finishing
  const char* bold = "\033[1m";
  const char* green = "\033[32m";
  const char* yellow = "\033[33m";
  const char* cyan = "\033[36m";
  const char* reset = "\033[0m";
  std::cout << std::endl;
  std::cout << bold << green << "[Subscriber finished]" << reset << std::endl;
  if (log_to_file) {
    std::cout << cyan << "Latency data has been saved to 'latency_log.csv'." << reset << std::endl;
    std::cout << "You can analyze the results with: " << yellow << "python3 /workspace/analyze_latency.py" << reset << std::endl;
  }
  return 0;
}
