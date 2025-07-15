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

#ifndef FLAT_ROS2_PUBLISHER_HPP
#define FLAT_ROS2_PUBLISHER_HPP

#include <cassert>
#include <memory>

#include <flatros2/message.hpp>

#include <rclcpp/node_interfaces/get_node_base_interface.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_options.hpp>

namespace flatros2 {

template <typename MessageT, typename AllocatorT = std::allocator<void>>
class FlatPublisher : public rclcpp::Publisher<as_flat_t<MessageT>, AllocatorT>
{
    using Base = rclcpp::Publisher<as_flat_t<MessageT>, AllocatorT>;
  public:
    template <typename NodeT>
    explicit FlatPublisher(NodeT* node, const std::string& topic_name, const rclcpp::QoS& qos_profile) 
      : Base(rclcpp::node_interfaces::get_node_base_interface(node).get(), topic_name,
             qos_profile, rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
    {
    }
};

template <typename MessageT, typename NodeT, typename AllocatorT = std::allocator<void>>
std::shared_ptr<FlatPublisher<MessageT, AllocatorT>> 
create_flat_publisher(NodeT* node, const std::string& topic_name, const rclcpp::QoS& qos_profile) {
  return std::make_shared<FlatPublisher<MessageT, AllocatorT>>(node, topic_name, qos_profile);
}

}  // namespace flatros2

#endif  // FLAT_ROS2_PUBLISHER_HPP