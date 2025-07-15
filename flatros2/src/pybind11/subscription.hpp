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

#ifndef FLAT_ROS2_PYBIND11_SUBSCRIPTION_HPP
#define FLAT_ROS2_PYBIND11_SUBSCRIPTION_HPP

#include <pybind11/pybind11.h>

#include <rcl/subscription.h>
#include <rcl/wait.h>

#include <cstddef>
#include <string>

#include "destroyable.hpp"

namespace py = pybind11;

namespace flatros2 {
namespace pybind11 {

class FlatSubscriptionImpl : public Destroyable {
  public:
    explicit FlatSubscriptionImpl(
      py::object pynode_handle,
      py::object pymessage_type,
      const std::string &topic_name,
      py::object pyqos_profile);

    ~FlatSubscriptionImpl() override;

    std::string get_topic_name() const;

    py::object take_loaned_message();

    void return_loaned_message(py::object pymessage);

    void add_to_waitset(py::object pywait_set);

    bool is_ready(py::object pywait_set);

  private:
    void destroy() override;

    py::object pynode_handle_;
    py::object pymessage_type_;
    rcl_subscription_t rcl_subscription_;
    size_t index_;
    bool ready_{false};
};

}  // namespace pybind11
}  // namespace flatros2

#endif  // FLAT_ROS2_PYBIND11_SUBSCRIPTION_HPP
