// Copyright 2025 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <flatros2/flatros2.hpp>
#include <std_msgs/msg/header.hpp>
#include <flatros2_test/msg/benchmark_data.hpp>

using namespace flatros2;

// Declare flat structure for builtin_interfaces::msg::Time
DECLARE_FLAT_STRUCTURE(builtin_interfaces::msg::Time) {
  FlatView<int32_t>& sec = this->template member_decl<int32_t>("sec");
  FlatView<uint32_t>& nanosec = this->template member_decl<uint32_t>("nanosec");
};

// Declare flat structure for std_msgs::msg::Header
DECLARE_FLAT_STRUCTURE(std_msgs::msg::Header) {
  FlatView<builtin_interfaces::msg::Time>& stamp = this->template member_decl<builtin_interfaces::msg::Time>("stamp");
};

// Declare flat prototype for benchmark_msgs::msg::BenchmarkData
DECLARE_FLAT_PROTOTYPE(flatros2_test::msg::BenchmarkData) {
  FlatView<std_msgs::msg::Header>& header = this->template member_decl<std_msgs::msg::Header>("header");
  FlatView<std::span<uint8_t>>& data = this->template member_decl<std::span<uint8_t>>("data");
};


// BenchmarkSubscriber node definition

class BenchmarkSubscriber : public rclcpp::Node {
public:
  BenchmarkSubscriber(bool log_to_file = true, bool debug = false)
    : Node("benchmark_subscriber", rclcpp::NodeOptions().start_parameter_services(false)),
      log_to_file_(log_to_file), debug_(debug)
  {
    if (log_to_file_) {
      logfile_.open("latency_log.csv");
      logfile_ << "[START] Program started\n";
      logfile_ << "size_bytes,latency_ms\n";
      logfile_.flush();
    }
    sub_ = create_flat_subscription<flatros2_test::msg::BenchmarkData>(
      this, "benchmark_topic", 10,
      [this](std::shared_ptr<Flat<flatros2_test::msg::BenchmarkData>> msg) {
        auto now = this->get_clock()->now();
        auto msg_time = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        double latency_ms = (now - msg_time).seconds() * 1e3;
        if (log_to_file_ && logfile_.is_open()) {
          logfile_ << msg->data.size() << "," << latency_ms << "\n";
          logfile_.flush();
        }
        if (debug_) {
          std::cout << "[DEBUG] Received message of size: " << msg->data.size()
                    << " bytes, latency: " << latency_ms << " ms" << std::endl;
        }
      });
  }

  ~BenchmarkSubscriber() {
    if (log_to_file_ && logfile_.is_open()) {
      logfile_ << "[END] Program exiting\n";
      logfile_.flush();
      logfile_.close();
    }
  }

private:
  std::ofstream logfile_;
  std::shared_ptr<FlatSubscription<flatros2_test::msg::BenchmarkData>> sub_;
  bool log_to_file_;
  bool debug_;
};

// Main function: initializes ROS 2, spins node, and shuts down
void print_help() {
  const char* bold = "\033[1m";
  const char* green = "\033[32m";
  const char* yellow = "\033[33m";
  const char* cyan = "\033[36m";
  const char* reset = "\033[0m";
  std::cout << bold << green << "Usage:" << reset << " benchmark_sub [--duration <seconds>] [--log true|false] [--debug] [--help]\n";
  std::cout << yellow << "  --duration, -d <seconds>" << reset << "  Time to run before exiting (default: run until killed)\n";
  std::cout << yellow << "  --log true|false     " << reset << "  Enable or disable logging to file (default: true)\n";
  std::cout << yellow << "  --debug              " << reset << "  Print each received message's size and latency to console\n";
  std::cout << yellow << "  --help, -h           " << reset << "  Show this help message\n";
  std::cout << "\n";
  std::cout << cyan << "This program subscribes to messages and logs latency data to 'latency_log.csv' (unless --log false).\n" << reset;
  std::cout << cyan << "You can analyze the latency data using the script 'analyze_latency.py'.\n" << reset;
  std::cout << bold << "Instructions to run the subscriber:" << reset << std::endl;
  std::cout << green << "  ./install/flatros2/lib/flatros2/benchmark_sub --duration 30" << reset << std::endl;
  std::cout << "This will start the subscriber and log latency data to 'latency_log.csv'." << std::endl;
  std::cout << "You can analyze the results with: " << yellow << "python3 /workspace/analyze_latency.py" << reset << std::endl;
  std::cout << std::endl;
  std::cout << bold << yellow << "Note:" << reset << " In another terminal, run the " << green << "publisher" << reset << " to send benchmark messages:" << std::endl;
  std::cout << "  " << cyan << "python3 src/flatros2/examples/benchmark_pub.py --mode sweep" << reset << std::endl;
  std::cout << "  (or use --mode duration)" << std::endl;
}


int main(int argc, char** argv) {
  // Warn user about how to kill the program
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
