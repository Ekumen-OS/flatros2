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

namespace {

struct Wrapper {
  uint8_t *data;
  size_t size;
};

void *wrap_message(uint8_t* payload, size_t size, void *type_erased_wrapper, bool copy) {
  assert(!type_erased_wrapper);
  assert(!copy);
  return new Wrapper{payload, size};
}

}  // namespace

py::object wrap_loaned_message(void *message, py::object pymessage_type) {
  using namespace py::literals;
  auto *wrapper = static_cast<Wrapper *>(message);
  auto buffer = py::memoryview::from_memory(wrapper->data, wrapper->size);
  auto pymessage = pymessage_type("_buffer"_a=buffer);
  delete wrapper;
  return pymessage;
}

void *unwrap_loaned_message(py::object pymessage, py::object pymessage_type) {
  if (!py::isinstance(pymessage, pymessage_type)) {
    throw std::runtime_error("not a loaned message");
  }
  auto pyview = pymessage.attr("_buffer").cast<py::memoryview>();
  void *loaned_message = PyMemoryView_GET_BUFFER(pyview.ptr())->buf;
  pyview.attr("release")();
  return loaned_message;
}

py::capsule make_flat_message_type_support(py::object pyprototype, std::optional<py::bytes> pyimage) {
  py::object pyproto_type_support = pyprototype.attr("_TYPE_SUPPORT");
  const auto *proto_message_type_support =
    static_cast<const rosidl_message_type_support_t *>(pyproto_type_support.cast<py::capsule>());
  auto flat_message_type_support = std::make_unique<rosidl_message_type_support_t>();
  flat_message_type_support->typesupport_identifier = typesupport_identifier;
  auto type_support = std::make_unique<flat_message_type_support_t>();
  std::unique_ptr<uint8_t[]> image;
  if (pyimage.has_value()) {
    auto view = static_cast<std::string_view>(*pyimage);
    image = std::make_unique<uint8_t[]>(view.size());
    std::copy(view.begin(), view.end(), image.get());
    type_support->message_image = image.get();
    type_support->message_size = view.size();
  } else {
    type_support->message_image = nullptr;
    type_support->message_size = 0u;
  }
  type_support->wrap_message = wrap_message; 
  type_support->unwrap_message = nullptr;  // no unwrap 
  flat_message_type_support->data = type_support.get();
  flat_message_type_support->func = get_message_typesupport_handle_function;
  flat_message_type_support->get_type_hash_func = proto_message_type_support->get_type_hash_func;
  flat_message_type_support->get_type_description_func = proto_message_type_support->get_type_description_func;
  flat_message_type_support->get_type_description_sources_func = proto_message_type_support->get_type_description_sources_func;
  auto capsule = py::capsule(flat_message_type_support.get(), +[](void *ptr) {
    auto *ts = static_cast<rosidl_message_type_support_t *>(ptr);
    auto *data = const_cast<flat_message_type_support_t *>(
      static_cast<const flat_message_type_support_t *>(ts->data));
    delete[] data->message_image;
    delete data;
    delete ts;
  });
  flat_message_type_support.release();
  type_support.release();
  image.release();
  return capsule;
}

} //  namespace pybind11
} //  namespace flatros2
