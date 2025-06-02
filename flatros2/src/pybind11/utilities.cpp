#include "utilities.hpp"
#include "typesupport.hpp"

#include <cstddef>

namespace flatros2 {
namespace pybind11 {

rcl_node_t *get_rcl_node(py::object pynode_handle) {
    return reinterpret_cast<rcl_node_t *>(pynode_handle.attr("pointer").cast<size_t>());
}

py::object wrap_loaned_message(void *loaned_message, py::object pymessage_type) {
  using namespace py::literals;
  const flat_message_type_support_t *ts = get_flat_message_type_support(pymessage_type);
  return pymessage_type("_buffer"_a=py::memoryview::from_memory(loaned_message, ts->message_size));
}

void *release_loaned_message(py::object pymessage, py::object pymessage_type) {
  if (!py::isinstance(pymessage, pymessage_type)) {
    throw std::runtime_error("not a loaned message");
  }
  auto pyview = pymessage.attr("_buffer").cast<py::memoryview>();
  void *loaned_message = PyMemoryView_GET_BUFFER(pyview.ptr())->buf;
  pyview.attr("release")();
  return loaned_message;
}

} // namespace pybind11
} // namespace flatros2
