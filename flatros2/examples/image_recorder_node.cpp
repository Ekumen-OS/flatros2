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

#include <functional>
#include <memory>
#include <span>
#include <string>

#include <flatros2/flatros2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <rosbag2_cpp/writer.hpp>

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
};

class ImageRecorderNode : public rclcpp::Node {
public: 
  explicit ImageRecorderNode() : rclcpp::Node(
    "image_recorder_node", rclcpp::NodeOptions().start_parameter_services(false)
                                                .start_parameter_event_publisher(false)
                                                .enable_logger_service(false)
                                                .enable_rosout(false))
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(rosbag2_storage::StorageOptions{"image_bag", "mcap"});
    sub_ = create_flat_subscription<sensor_msgs::msg::Image>(
      this, "image", 10, std::bind(&ImageRecorderNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(std::shared_ptr<Flat<sensor_msgs::msg::Image>> input_message) const {    
    const auto stamp = rclcpp::Time(
       input_message->header.stamp.sec,
       input_message->header.stamp.nanosec, 
       RCL_ROS_TIME);
    auto serialized_message = std::make_shared<rclcpp::SerializedMessage>();
    serialized_message->get_rcl_serialized_message() = static_cast<rcl_serialized_message_t>(*input_message);
    writer_->write(serialized_message, sub_->get_topic_name(), "sensor_msgs/msg/Image", stamp);
    serialized_message->release_rcl_serialized_message();
  }

  std::shared_ptr<FlatSubscription<sensor_msgs::msg::Image>> sub_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
