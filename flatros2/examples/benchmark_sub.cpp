#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <flatros2/flatros2.hpp>
#include <std_msgs/msg/header.hpp>
#include <benchmark_msgs/msg/benchmark_data.hpp>

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
DECLARE_FLAT_PROTOTYPE(benchmark_msgs::msg::BenchmarkData) {
  FlatView<std_msgs::msg::Header>& header = this->template member_decl<std_msgs::msg::Header>("header");
  FlatView<std::span<uint8_t>>& data = this->template member_decl<std::span<uint8_t>>("data");
};


// BenchmarkSubscriber node definition
class BenchmarkSubscriber : public rclcpp::Node {
public:
  // Constructor: sets up log file and subscription
  BenchmarkSubscriber()
  : Node("benchmark_subscriber", rclcpp::NodeOptions().start_parameter_services(false)) {
    // Open log file for latency results
    logfile_.open("latency_log.csv");
    logfile_ << "[START] Program started\n";
    logfile_ << "size_bytes,latency_ms\n";
    logfile_.flush();

    // Create flat subscription to benchmark_topic
    sub_ = create_flat_subscription<benchmark_msgs::msg::BenchmarkData>(
      this, "benchmark_topic", 10,
      [this](std::shared_ptr<Flat<benchmark_msgs::msg::BenchmarkData>> msg) {
        // Callback: compute latency and log to file
        auto now = this->get_clock()->now();
        auto msg_time = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        double latency_ms = (now - msg_time).seconds() * 1e3;

        auto data_span = static_cast<std::span<uint8_t>>(msg->data);
        logfile_ << data_span.size() << "," << latency_ms << "\n";
        logfile_.flush();
        // RCLCPP_INFO(this->get_logger(), "Received message of size: %zu bytes, latency: %.2f ms", data_span.size(), latency_ms);
      });
  }

  // Destructor: closes log file
  ~BenchmarkSubscriber() {
    if (logfile_.is_open()) {
      logfile_ << "[END] Program exiting\n";
      logfile_.flush();
      logfile_.close();
    }
  }

private:
  // Log file for latency results
  std::ofstream logfile_;
  // Flat subscription handle
  std::shared_ptr<FlatSubscription<benchmark_msgs::msg::BenchmarkData>> sub_;
};

// Main function: initializes ROS 2, spins node, and shuts down
void print_help() {
  const char* bold = "\033[1m";
  const char* green = "\033[32m";
  const char* yellow = "\033[33m";
  const char* cyan = "\033[36m";
  const char* reset = "\033[0m";
  std::cout << bold << green << "Usage:" << reset << " benchmark_sub [--duration <seconds>] [--help]\n";
  std::cout << yellow << "  --duration, -d <seconds>" << reset << "  Time to run before exiting (default: run until killed)\n";
  std::cout << yellow << "  --help, -h           " << reset << "  Show this help message\n";
  std::cout << "\n";
  std::cout << cyan << "This program subscribes to messages and logs latency data to 'latency_log.csv'.\n" << reset;
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
  int duration_sec = -1;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_help();
      return 0;
    } else if ((arg == "--duration" || arg == "-d") && i + 1 < argc) {
      duration_sec = std::stoi(argv[++i]);
    }
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BenchmarkSubscriber>();
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
  std::cout << cyan << "Latency data has been saved to 'latency_log.csv'." << reset << std::endl;
  std::cout << "You can analyze the results with: " << yellow << "python3 /workspace/analyze_latency.py" << reset << std::endl;
  return 0;
}
