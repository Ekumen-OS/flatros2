#ifndef FLAT_ROS2_PYBIND11_TYPESUPPORT_HPP
#define FLAT_ROS2_PYBIND11_TYPESUPPORT_HPP

#include <pybind11/pybind11.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <flatros2/typesupport.hpp>

namespace py = pybind11;

namespace flatros2 {
namespace pybind11 {

const rosidl_message_type_support_t *get_message_type_support(py::object pymessage_type);

const flat_message_type_support_t *get_flat_message_type_support(py::object pymessage_type);

py::bytes get_flat_message_image(py::object pymessage_type);

py::capsule make_flat_message_type_support(py::object pyprototype, py::bytes pyimage);

}  // namespace pybind11
}  // namespace flatros2

#endif  // FLAT_ROS2_PYBIND11_TYPESUPPORT_HPP
