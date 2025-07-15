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

#include "subscription.hpp"

#include "typesupport.hpp"
#include "utilities.hpp"

#include <rmw/types.h>

#include <rcl/types.h>
#include <rcl/subscription.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <stdexcept>

namespace flatros2 {
namespace pybind11 {

FlatSubscriptionImpl::FlatSubscriptionImpl(
  py::object pynode_handle, py::object pymessage_type,
  const std::string &topic_name, py::object pyqos_profile)
: pynode_handle_{std::move(pynode_handle)},
  pymessage_type_{std::move(pymessage_type)},
  rcl_subscription_{rcl_get_zero_initialized_subscription()}
{
  pynode_handle_.attr("__enter__")();
  rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
  subscription_options.qos = pyqos_profile.cast<rmw_qos_profile_t>();
  subscription_options.disable_loaned_message = false;
  rcl_ret_t ret = rcl_subscription_init(
    &rcl_subscription_, get_rcl_node(pynode_handle_),
    get_message_type_support(pymessage_type_),
    topic_name.c_str(), &subscription_options);
  if (RCL_RET_OK != ret) {
    pynode_handle_.attr("__exit__")(py::none(), py::none(), py::none());
    throw std::runtime_error("failed to initialize subscription");
  }
}

FlatSubscriptionImpl::~FlatSubscriptionImpl() { destroy(); }

void FlatSubscriptionImpl::destroy() {
  if (rcl_subscription_is_valid(&rcl_subscription_)) {
    rcl_ret_t ret = rcl_subscription_fini(&rcl_subscription_, get_rcl_node(pynode_handle_));
    if (RCL_RET_OK != ret) {
      throw std::runtime_error("failed to finalize subscription");
    }
    pynode_handle_.attr("__exit__")(py::none(), py::none(), py::none());
  }
}

std::string FlatSubscriptionImpl::get_topic_name() const {
  const char * topic_name = rcl_subscription_get_topic_name(&rcl_subscription_);
  if (!topic_name) {
    throw std::runtime_error("failed to get subscription topic name");
  }
  return std::string(topic_name);
}

py::object FlatSubscriptionImpl::take_loaned_message() {
  if (!ready_) {
    return py::none();
  }
  ready_ = false;
  void * message = nullptr;
  rcl_ret_t ret = rcl_take_loaned_message(&rcl_subscription_, &message, nullptr, nullptr);
  if (RCL_RET_OK != ret) {
    if (RCL_RET_BAD_ALLOC == ret) {
      throw std::bad_alloc();
    }
    if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
      return py::none();
    }
    if (RCL_RET_OK != ret) {
      throw std::runtime_error("failed to take loaned message");
    }
  }
  return wrap_loaned_message(message, pymessage_type_);
}

void FlatSubscriptionImpl::return_loaned_message(py::object pymessage) {
  void * message = unwrap_loaned_message(pymessage, pymessage_type_);
  rcl_ret_t ret = rcl_return_loaned_message_from_subscription(&rcl_subscription_, message);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to return loaned message");
  }
}

void FlatSubscriptionImpl::add_to_waitset(py::object pywait_set) {
  auto *wait_set =
    reinterpret_cast<rcl_wait_set_t *>(pywait_set.attr("pointer").cast<size_t>());
  rcl_ret_t ret = rcl_wait_set_add_subscription(wait_set, &rcl_subscription_, &index_);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to add subscription to wait set");
  }
}

bool FlatSubscriptionImpl::is_ready(py::object pywait_set) {
  const auto *wait_set =
    reinterpret_cast<const rcl_wait_set_t *>(pywait_set.attr("pointer").cast<size_t>());
  if (index_ >= wait_set->size_of_subscriptions) {
    throw std::out_of_range("wait set index too big");
  }
  ready_ = (wait_set->subscriptions[index_] != nullptr);
  return ready_;
}

}  // namespace pybind11
}  // namespace flatros2
