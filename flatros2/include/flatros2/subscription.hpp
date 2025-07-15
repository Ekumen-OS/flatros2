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

#ifndef FLAT_ROS2_SUBSCRIPTION_HPP
#define FLAT_ROS2_SUBSCRIPTION_HPP

#include <utility>

#include <flatros2/message.hpp>

#include <rclcpp/any_subscription_callback.hpp>

#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>

namespace flatros2 {

template <typename MessageT, typename AllocatorT = std::allocator<void>>
class FlatSubscription : public rclcpp::Subscription<as_flat_t<MessageT>, AllocatorT> 
{
    using Base = rclcpp::Subscription<as_flat_t<MessageT>, AllocatorT>;

    template<typename CallbackT>
    rclcpp::AnySubscriptionCallback<typename Base::SubscribedType, AllocatorT> wrap_callback(CallbackT callback) {
        rclcpp::AnySubscriptionCallback<typename Base::SubscribedType, AllocatorT> wrapper;
        wrapper.set(std::move(callback));
        return wrapper;
    }

  public:
    template <typename NodeT, typename CallbackT>
    explicit FlatSubscription(NodeT* node, const std::string& topic_name, const rclcpp::QoS& qos_profile, CallbackT&& callback) 
      : Base(rclcpp::node_interfaces::get_node_base_interface(node).get(), 
             rclcpp::get_message_type_support_handle<typename Base::SubscribedType>(), 
             topic_name, qos_profile, wrap_callback(std::forward<CallbackT>(callback)), 
             rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(), 
             Base::MessageMemoryStrategyType::create_default())
    {
    }
};

template <typename MessageT, typename NodeT, typename CallbackT, typename AllocatorT = std::allocator<void>>
std::shared_ptr<FlatSubscription<MessageT, AllocatorT>>
create_flat_subscription(NodeT* node, const std::string& topic_name, const rclcpp::QoS& qos_profile, CallbackT&& callback) {
  auto subscription = std::make_shared<FlatSubscription<MessageT, AllocatorT>>(node, topic_name, qos_profile, std::forward<CallbackT>(callback));
  auto node_topics = rclcpp::node_interfaces::get_node_topics_interface(node);
  node_topics->add_subscription(subscription, nullptr);
  return subscription;
}

}  // namespace flatros2

#endif  // FLAT_ROS2_SUBSCRIPTION_HPP