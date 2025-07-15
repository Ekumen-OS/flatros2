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

#include "publisher.hpp"

#include "typesupport.hpp"
#include "utilities.hpp"

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rcl/error_handling.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <stdexcept>

namespace flatros2 {
namespace pybind11 {

FlatPublisherImpl::FlatPublisherImpl(
  py::object pynode_handle, py::object pymessage_type,
  const std::string &topic_name, const py::object pyqos_profile)
: pynode_handle_{std::move(pynode_handle)},
  pymessage_type_{std::move(pymessage_type)},
  rcl_publisher_{rcl_get_zero_initialized_publisher()}
{
  pynode_handle_.attr("__enter__")();
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  publisher_options.qos = pyqos_profile.cast<rmw_qos_profile_t>();
  rcl_ret_t ret = rcl_publisher_init(
    &rcl_publisher_, get_rcl_node(pynode_handle_),
    get_message_type_support(pymessage_type_),
    topic_name.c_str(), &publisher_options);
  if (RCL_RET_OK != ret) {
    pynode_handle_.attr("__exit__")(py::none(), py::none(), py::none());
    throw std::runtime_error("failed to initialized publisher");
  }
}

FlatPublisherImpl::~FlatPublisherImpl() { destroy(); }

void FlatPublisherImpl::destroy() {
  if (rcl_publisher_is_valid(&rcl_publisher_)) {
    rcl_ret_t ret = rcl_publisher_fini(&rcl_publisher_, get_rcl_node(pynode_handle_));
    if (RCL_RET_OK != ret) {
      throw std::runtime_error("failed to initialized publisher");
    }
    pynode_handle_.attr("__exit__")(py::none(), py::none(), py::none());
  }
}

std::string FlatPublisherImpl::get_topic_name() const {
  const char * topic_name = rcl_publisher_get_topic_name(&rcl_publisher_);
  if (!topic_name) {
    throw std::runtime_error("failed to get topic name");
  }
  return std::string(topic_name);
}

py::object FlatPublisherImpl::borrow_loaned_message() {
  void * message = nullptr;
  const rosidl_message_type_support_t *ts = get_message_type_support(pymessage_type_);
  rcl_ret_t ret = rcl_borrow_loaned_message(&rcl_publisher_, ts, &message);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to borrow message");
  }
  return wrap_loaned_message(message, pymessage_type_);
}

void FlatPublisherImpl::publish_loaned_message(py::object pymessage) {
  void * message = unwrap_loaned_message(pymessage, pymessage_type_);
  rcl_ret_t ret = rcl_publish_loaned_message(&rcl_publisher_, message, nullptr);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to publish loaned message");
  }
}

void FlatPublisherImpl::return_loaned_message(py::object pymessage) {
  void * message = unwrap_loaned_message(pymessage, pymessage_type_);
  rcl_ret_t ret = rcl_return_loaned_message_from_publisher(&rcl_publisher_, &message);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to return loaned message");
  }
}

} // namespace pybind11
} // namespace flatros2
