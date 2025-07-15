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

#ifndef FLAT_ROS2_PYBIND11_TYPESUPPORT_HPP
#define FLAT_ROS2_PYBIND11_TYPESUPPORT_HPP

#include <optional>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <flatros2/typesupport.hpp>

namespace py = pybind11;

namespace flatros2 {
namespace pybind11 {

const rosidl_message_type_support_t *get_message_type_support(py::object pymessage_type);

const flat_message_type_support_t *get_flat_message_type_support(py::object pymessage_type);

py::bytes get_flat_message_image(py::object pymessage_type);

py::capsule make_flat_message_type_support(py::object pyprototype, std::optional<py::bytes> pyimage);

py::object wrap_loaned_message(void *message, py::object pymessage_type);

void *unwrap_loaned_message(py::object pymessage, py::object pymessage_type);

}  // namespace pybind11
}  // namespace flatros2

#endif  // FLAT_ROS2_PYBIND11_TYPESUPPORT_HPP
