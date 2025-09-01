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

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <flatros2/flatros2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace flatros2;

DECLARE_FLAT_STRUCTURE(builtin_interfaces::msg::Time) {
  FlatView<int32_t>& sec = this->template member_decl<int32_t>("sec");
  FlatView<uint32_t>& nanosec = this->template member_decl<uint32_t>("nanosec");
};

DECLARE_FLAT_STRUCTURE(std_msgs::msg::Header) {
  FlatView<builtin_interfaces::msg::Time>& stamp = this->template member_decl<builtin_interfaces::msg::Time>("stamp");
};

DECLARE_FLAT_PROTOTYPE(sensor_msgs::msg::Image) {
  // NOTE: no auto allowed in non-static data members :(
  FlatView<std_msgs::msg::Header>& header = this->template member_decl<std_msgs::msg::Header>("header");
  FlatView<std::string>& encoding = this->template member_decl<std::string>("encoding");
  FlatView<uint32_t>& width = this->template member_decl<uint32_t>("width");
  FlatView<uint32_t>& height = this->template member_decl<uint32_t>("height");
  FlatView<uint32_t>& step = this->template member_decl<uint32_t>("step");
  FlatView<std::span<uint8_t>>& data = this->template member_decl<std::span<uint8_t>>("data");  
};

cv::Mat getCvMat(const Flat<sensor_msgs::msg::Image>& message) {
  const int message_type = cv_bridge::getCvType(message.encoding);
  auto message_data = const_cast<unsigned char *>(message.data.data());
  return cv::Mat(message.height, message.width, message_type, message_data, message.step);
}

class EdgeDetectorNode : public rclcpp::Node {
public: 
  explicit EdgeDetectorNode() : rclcpp::Node(
    "edge_detector_node", rclcpp::NodeOptions().start_parameter_services(false)
                                               .start_parameter_event_publisher(false)
                                               .enable_logger_service(false)
                                               .enable_rosout(false))
  {
    sub_ = create_flat_subscription<sensor_msgs::msg::Image>(
      this, "input_image", 10, std::bind(&EdgeDetectorNode::callback, this, std::placeholders::_1));
    pub_ = create_flat_publisher<sensor_msgs::msg::Image>(this, "output_image", 10);
  }

private:
  void callback(std::shared_ptr<Flat<sensor_msgs::msg::Image>> input_message) const {    
    rclcpp::Time now = this->get_clock()->now();
    const auto message_stamp = rclcpp::Time(
       input_message->header.stamp.sec,
       input_message->header.stamp.nanosec, 
       RCL_ROS_TIME);
    const auto delay = now - message_stamp;
    std::cout << "delay = " << delay.seconds() << std::endl;

    cv::Mat rgb_image = getCvMat(*input_message);

    cv::Mat gray_image;
    cv::cvtColor(rgb_image, gray_image, cv::COLOR_RGB2GRAY);

    cv::Mat gray_edges;
    cv::Canny(gray_image, gray_edges, 100, 200);

    auto output_message = pub_->borrow_loaned_message();
    output_message.get() = *input_message;
    cv::Mat output_rgb_image = getCvMat(output_message.get());
    cv::cvtColor(gray_edges, output_rgb_image, cv::COLOR_GRAY2RGB);

    pub_->publish(std::move(output_message));
  }

  std::shared_ptr<FlatSubscription<sensor_msgs::msg::Image>> sub_;
  std::shared_ptr<FlatPublisher<sensor_msgs::msg::Image>> pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}