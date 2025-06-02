#ifndef FLAT_ROS2_PYBIND11_UTILITIES_HPP
#define FLAT_ROS2_PYBIND11_UTILITIES_HPP

#include <pybind11/pybind11.h>

#include <rcl/node.h>

namespace py = pybind11;

namespace flatros2 {
namespace pybind11 {

rcl_node_t *get_rcl_node(py::object pynode_handle);

py::object wrap_loaned_message(void *loaned_message, py::object pymessage_type);

void *release_loaned_message(py::object pymessage, py::object pymessage_type);

}  // namespace pybind11
}  // namespace flatros2

#endif  // FLAT_ROS2_PYBIND11_UTILITIES_HPP
