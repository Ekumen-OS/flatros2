#include "typesupport.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace flatros2 {
namespace pybind11 {

const rosidl_message_type_support_t *get_message_type_support(py::object pymessage_type) {
  py::object pymessage_ts = pymessage_type.attr("_TYPE_SUPPORT");
  return static_cast<const rosidl_message_type_support_t *>(pymessage_ts.cast<py::capsule>());
}

const flat_message_type_support_t *get_flat_message_type_support(py::object pymessage_type) {
  const rosidl_message_type_support_t *ts = get_message_typesupport_handle(
    get_message_type_support(pymessage_type), typesupport_identifier);
  if (!ts) {
    throw std::runtime_error("no flat type support found");
  }
  return static_cast<const flat_message_type_support_t *>(ts->data);
}

py::bytes get_flat_message_image(py::object pymessage_type) {
  const auto *ts = get_flat_message_type_support(pymessage_type);
  return py::bytes(reinterpret_cast<const char *>(ts->message_image), ts->message_size);
}

py::capsule make_flat_message_type_support(py::object pyprototype, py::bytes pyimage) {
  py::object pyprototype_class = pyprototype.attr("__class__");
  pyprototype_class.attr("__import_type_support__")();
  py::object pyprototype_ts = pyprototype_class.attr("_TYPE_SUPPORT");
  const auto *prototype_ts =
    static_cast<const rosidl_message_type_support_t *>(pyprototype_ts.cast<py::capsule>());
  auto view = static_cast<std::string_view>(pyimage);
  auto image = std::make_unique<uint8_t[]>(view.size());
  std::copy(view.begin(), view.end(), image.get());
  auto data = std::make_unique<flat_message_type_support_t>();
  auto ts = std::make_unique<rosidl_message_type_support_t>();
  data->message_image = image.get();
  data->message_size = view.size();
  ts->typesupport_identifier = typesupport_identifier;
  ts->data = data.get();
  ts->func = get_message_typesupport_handle_function;
  ts->get_type_hash_func = prototype_ts->get_type_hash_func;
  ts->get_type_description_func = prototype_ts->get_type_description_func;
  ts->get_type_description_sources_func = prototype_ts->get_type_description_sources_func;
  auto capsule = py::capsule(ts.get(), +[](void *ptr) {
    auto *ts = static_cast<rosidl_message_type_support_t *>(ptr);
    auto *data = const_cast<flat_message_type_support_t *>(
      static_cast<const flat_message_type_support_t *>(ts->data));
    delete[] data->message_image;
    delete data;
    delete ts;
  });
  ts.release();
  data.release();
  image.release();
  return capsule;
}

} //  namespace pybind11
} //  namespace flatros2
