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

#ifndef FLAT_ROS2_VIEW_HPP
#define FLAT_ROS2_VIEW_HPP

#include <cstddef>
#include <cstdint>
#include <codecvt>
#include <locale>
#include <span>
#include <string>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_cpp/traits.hpp>

#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

#include <flatbuffers/flexbuffers.h>

namespace flatros2 {

template <typename MessageT, typename DerivedT>
struct Flat;  // Forward declaration

template <typename T>
struct is_flat : std::false_type {};

template <typename MessageT, typename DerivedT>
struct is_flat<Flat<MessageT, DerivedT>> : std::true_type {};

template <typename T>
constexpr bool is_flat_v = is_flat<T>::value;

template <typename T>
struct is_flat_message : std::false_type {};

template <typename MessageT>
struct is_flat_message<Flat<MessageT, void>> : std::false_type {};

template <typename MessageT, typename DerivedT>
struct is_flat_message<Flat<MessageT, DerivedT>> : std::true_type {};

template <typename T>
constexpr bool is_flat_message_v = is_flat_message<T>::value;

struct FlatViewInterface {
  virtual ~FlatViewInterface() = default;

  virtual void reset() = 0;
  virtual void reset(flexbuffers::Reference ref) = 0;
};

template <typename T, typename = void>
struct FlatView;

template <>
struct FlatView<bool> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { ref_ = flexbuffers::Reference(); }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsBool()) {
      throw std::runtime_error("underlying value is not a boolean");
    }
    ref_ = ref;
  }

  operator bool() const { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return ref_.AsBool(); 
  }

  FlatView& operator=(bool other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    if (!ref_.MutateBool(other)) {
      throw std::runtime_error("failed to assign boolean");
    }
    return *this;
  }
private:
  flexbuffers::Reference ref_;
};

template <>
struct FlatView<char> : FlatViewInterface{
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }
  
  void reset() override { 
    ref_ = flexbuffers::Reference();
    blob_ = flexbuffers::Blob::EmptyBlob();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsBlob()) {
      throw std::runtime_error("underlying value is not a character");
    }
    flexbuffers::Blob blob = ref.AsBlob();
    if (blob.size() != 1) {
      throw std::runtime_error("underlying value is not a character");
    }
    ref_ = ref;
    blob_ = blob;
  }

  operator char() const { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return static_cast<char>(*blob_.data()); 
  }

  FlatView& operator=(char other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    char * data = reinterpret_cast<char *>(
      const_cast<uint8_t *>(blob_.data()));
    *data = other;
    return *this;
  }
private:
  flexbuffers::Reference ref_;
  flexbuffers::Blob blob_{flexbuffers::Blob::EmptyBlob()};
};

template <>
struct FlatView<char16_t> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { 
    ref_ = flexbuffers::Reference();
    blob_ = flexbuffers::Blob::EmptyBlob();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsBlob()) {
      throw std::runtime_error("underlying value is not a character");
    }
    ref_ = ref;
  }

  operator char16_t() const { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    const char* first = reinterpret_cast<const char *>(blob_.data());
    const char* last = 
      reinterpret_cast<const char *>(blob_.data() + blob_.size());
    const std::u16string string = utf8_codec_.from_bytes(first, last);
    if (string.size() != 1) {
      throw std::runtime_error("underlying value is not a single character");
    }
    return string.front(); 
  }

  FlatView& operator=(char16_t other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    const std::string bytes = utf8_codec_.to_bytes(other);
    if (bytes.size() != blob_.size()) {
      throw std::runtime_error("failed to assign character");
    }
    char * data = reinterpret_cast<char *>(
      const_cast<uint8_t *>(blob_.data()));
    bytes.copy(data, bytes.size());
    return *this;
  }
private:
  static std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> utf8_codec_;

  flexbuffers::Reference ref_;
  flexbuffers::Blob blob_{flexbuffers::Blob::EmptyBlob()};
};

template <typename IntegerT>
struct FlatView<IntegerT, std::enable_if_t<
    std::is_same_v<IntegerT, int8_t> || std::is_same_v<IntegerT, int16_t> ||
    std::is_same_v<IntegerT, int32_t> || std::is_same_v<IntegerT, int64_t>>> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }
  
  void reset() override { 
    ref_ = flexbuffers::Reference();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsInt()) {
      throw std::runtime_error("underlying value is not a signed integer");
    }
    ref_ = ref;
  }

  operator IntegerT() const { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return static_cast<IntegerT>(ref_.AsInt64()); 
  }

  FlatView& operator=(IntegerT other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    if (!ref_.MutateInt(other)) {
      throw std::runtime_error("failed to assign signed integer");
    }
    return *this;
  }
private:
  flexbuffers::Reference ref_;
};

template <typename UIntegerT>
struct FlatView<UIntegerT, std::enable_if_t<
    std::is_same_v<UIntegerT, uint8_t> || std::is_same_v<UIntegerT, uint16_t> ||
    std::is_same_v<UIntegerT, uint32_t> || std::is_same_v<UIntegerT, uint64_t>>> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { 
    ref_ = flexbuffers::Reference();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsUInt()) {
      throw std::runtime_error("underlying value is not an unsigned integer");
     }
     ref_ = ref;
  }

  operator UIntegerT() const { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return static_cast<UIntegerT>(ref_.AsUInt64()); 
  }

  FlatView& operator=(UIntegerT other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    if (!ref_.MutateUInt(other)) {
      throw std::runtime_error("failed to assign unsigned integer");
    }
    return *this;
  }
private:
  flexbuffers::Reference ref_;
};

template <typename FloatT>
struct FlatView<FloatT, std::enable_if_t<std::is_floating_point_v<FloatT>>> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { 
    ref_ = flexbuffers::Reference();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsFloat()) {
      throw std::runtime_error("underlying value is not a floating point number");
    }
    ref_ = ref;
  }

  operator FloatT() const {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return static_cast<FloatT>(ref_.AsDouble()); 
  }

  FlatView& operator=(FloatT other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    if (!ref_.MutateFloat(other)) {
      throw std::runtime_error("failed to assign floating point number");
    }
    return *this;
  }
private:
  flexbuffers::Reference ref_;
};

template <>
struct FlatView<std::string> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { 
    ref_ = flexbuffers::Reference();
    blob_ = flexbuffers::Blob::EmptyBlob();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsBlob()) {
      throw std::runtime_error("underlying value is not a string");
     }
     ref_ = ref;
     blob_ = ref_.AsBlob();
  }

  operator std::string_view() const {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    const char *data = reinterpret_cast<const char *>(blob_.data());
    return std::string_view(data, blob_.size());
  }

  operator std::string() const {
    return std::string(std::string_view(*this));
  }

  FlatView& operator=(std::string_view other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    if (blob_.size() != other.size()) {
      throw std::runtime_error("failed to assign string");
    }
    char * data = reinterpret_cast<char *>(
      const_cast<uint8_t *>(blob_.data()));
    other.copy(data, other.length());
    return *this;
  }

private:
  flexbuffers::Reference ref_;
  flexbuffers::Blob blob_{flexbuffers::Blob::EmptyBlob()};
};

template <>
struct FlatView<std::u16string> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { 
    ref_ = flexbuffers::Reference();
    blob_ = flexbuffers::Blob::EmptyBlob();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsBlob()) {
      throw std::runtime_error("underlying value is not a string");
     }
     ref_ = ref;
     blob_ = ref_.AsBlob();
  }

  operator std::u16string() const {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    const char *first = reinterpret_cast<const char *>(blob_.data());
    return utf8_codec_.from_bytes(first, first + blob_.size());
  }

  FlatView& operator=(std::u16string_view other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    const std::string bytes = 
      utf8_codec_.to_bytes(other.data(), other.data() + other.size());
    if (blob_.size() != bytes.size()) {
      throw std::runtime_error("failed to assign string");
    }
    char * data = reinterpret_cast<char *>(
      const_cast<uint8_t *>(blob_.data()));
    bytes.copy(data, bytes.size());
    return *this;
  }

private:
  static std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> utf8_codec_;
  
  flexbuffers::Reference ref_;
  flexbuffers::Blob blob_{flexbuffers::Blob::EmptyBlob()};
};

template <typename T, size_t N>
struct FlatView<std::span<T, N>, std::enable_if_t<std::is_integral_v<T> || std::is_floating_point_v<T>>> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }
  
  void reset() override {
    ref_ = flexbuffers::Reference();
    blob_ = flexbuffers::Blob::EmptyBlob();
    count_ = 0u;
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsBlob()) {
      throw std::runtime_error("underlying value is not a span");
    }
    flexbuffers::Blob blob = ref.AsBlob();
    size_t count = blob.size() / sizeof(T); 
    if (N != std::dynamic_extent && N != count) {
      throw std::runtime_error("span size does not match that of the underlying");
    }
    ref_ = ref;
    blob_ = blob;
    count_ = count; 
  }

  T* data() { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return reinterpret_cast<T *>(const_cast<uint8_t *>(blob_.data())); 
  }
  
  const T* data() const {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    } 
    return reinterpret_cast<const T *>(blob_.data()); 
  }

  size_t size() const { return count_; }

  operator std::span<T, N>() { 
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return std::span<T, N>(data(), count_); 
  }

  FlatView& operator=(const std::span<T, N>& other) {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    if (other.size() != count_) {
      throw std::runtime_error("span size does not match that of the underlying");
    }
    std::copy(other.begin(), other.end(), data());
    return *this;
  }

private:
  flexbuffers::Reference ref_;
  flexbuffers::Blob blob_{flexbuffers::Blob::EmptyBlob()};
  size_t count_{0u};
};

template <typename T, size_t N>
struct FlatView<std::span<T, N>, std::enable_if_t<!(std::is_integral_v<T> || std::is_floating_point_v<T>)>> : FlatViewInterface {
public:
  FlatView() = default;

  explicit FlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override { 
    ref_ = flexbuffers::Reference();
    item_views_.clear();
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsVector()) {
      throw std::runtime_error("underlying value is not a span");
    }
    flexbuffers::Vector vector = ref.AsVector();
    if (N != std::dynamic_extent && N != vector.size()) {
      throw std::runtime_error("span size does not match that of the underlying value");
    }
    std::vector<FlatView<T>> item_views;
    item_views.reserve(vector.size());
    for (size_t i = 0; i < vector.size(); ++i) {
      item_views.emplace_back(vector[i]);
    }
    item_views_ = std::move(item_views);
    ref_ = ref;
  }

  size_t size() const { return item_views_.size(); }

  operator std::span<FlatView<T>, N>() {
    if (ref_.IsNull()) {
      throw std::runtime_error("void view");
    }
    return std::span(item_views_); 
  }

  FlatView& operator=(const std::span<T, N>& other) {
    if (ref_.IsNull()) { 
      throw std::runtime_error("void view");
    } 
    if (other.size() != item_views_.size()) {
      throw std::runtime_error("span size does not match that of the underlying value");
    }
    for (size_t i = 0; i < item_views_.size(); ++i) item_views_[i] = other[i];
    return *this;
  }

private:
  flexbuffers::Reference ref_;
  std::vector<FlatView<T>> item_views_;
};

namespace detail {

template <typename MessageT>
std::enable_if_t<
    rosidl_generator_traits::is_message<MessageT>::value && !is_flat_v<MessageT>,
    std::map<std::string, size_t>
> make_flat_message_layout() {
  const rosidl_message_type_support_t * type_support = 
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<MessageT>();
  const rosidl_message_type_support_t * introspection_type_support = 
    get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  const auto *message_description = 
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_type_support->data);
  std::map<std::string, size_t> layout;
  for (size_t i = 0; i < message_description->member_count_; ++i) {
    layout[message_description->members_[i].name_] = i;
  }
  return layout;
}

template <typename MessageT>
std::enable_if_t<
    rosidl_generator_traits::is_message<MessageT>::value && !is_flat_v<MessageT>, 
    const std::map<std::string, size_t>&
> get_flat_message_layout() {
  static std::map<std::string, size_t> layout = make_flat_message_layout<MessageT>();
  return layout;
}

template <typename MessageT>
struct MessageFlatView : FlatViewInterface {
public:
  using ProtoT = MessageT;

  MessageFlatView() = default;

  explicit MessageFlatView(flexbuffers::Reference ref) { reset(ref); }

  void reset() override {
    ref_ = flexbuffers::Reference();
    for (auto& [_, view] : member_views_) {
      view->reset();
    }
  }

  void reset(flexbuffers::Reference ref) override {
    if (!ref.IsVector()) {
      throw std::runtime_error("underlying value is not a flattened message");
    }
    flexbuffers::Vector items = ref.AsVector();
    const auto& layout = detail::get_flat_message_layout<MessageT>();
    if (layout.size() != items.size()) {
      throw std::runtime_error("unexpected message layout");
    }
    ref_ = std::move(ref);
    for (auto& [name, view] : member_views_) {
      view->reset(items[layout.at(name)]);
    }
  }

  template <typename T>
  FlatView<T>& member(const std::string& name) {
    if (!member_views_.contains(name)) {
      const auto& layout = detail::get_flat_message_layout<MessageT>();
      auto it = layout.find(name);
      if (it == layout.end()) {
        throw std::runtime_error("not a message member");
      }
      if (!ref_.IsNull()) {
        flexbuffers::Vector items = ref_.AsVector();
        member_views_[name] = std::make_shared<FlatView<T>>(items[it->second]);
      } else {
        member_views_[name] = std::make_shared<FlatView<T>>();
      }
    }
    return dynamic_cast<FlatView<T>&>(*member_views_[name]);
  }

private:
  flexbuffers::Reference ref_;
  std::map<std::string, std::shared_ptr<FlatViewInterface>> member_views_;
};

template <typename MessageT, typename DerivedT>
struct FlatStruct {};

template <typename DerivedT>
struct FlatStructOps {
  template <typename T>
  FlatView<T>& member_decl(const std::string& name) {
    auto *self = static_cast<DerivedT*>(this);
    return self->template member<T>(name);
  }
};

}  // namespace detail

template<typename MessageT>
struct FlatView<MessageT, std::enable_if_t<
    rosidl_generator_traits::is_message<MessageT>::value && !is_flat_v<MessageT>
>> : detail::MessageFlatView<MessageT>, detail::FlatStruct<MessageT, FlatView<MessageT>>  {
  using detail::MessageFlatView<MessageT>::MessageFlatView;
};

}  // namespace flatros2

#define DECLARE_FLAT_STRUCTURE(Name) \
  template<typename DerivedT> \
  struct flatros2::detail::FlatStruct<Name, DerivedT> : flatros2::detail::FlatStructOps<DerivedT>

#endif  // FLAT_ROS2_VIEW_HPP