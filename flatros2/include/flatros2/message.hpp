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

#ifndef FLAT_ROS2_MESSAGE_HPP
#define FLAT_ROS2_MESSAGE_HPP

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <variant>
#include <vector>

#include <flatbuffers/flexbuffers.h>

#include <flatros2/typesupport.hpp>
#include <flatros2/view.hpp>

#include <rclcpp/get_message_type_support_handle.hpp>
#include <rclcpp/is_ros_compatible_type.hpp>

namespace flatros2 {

flexbuffers::Builder& flatten_message(
    flexbuffers::Builder& builder, const void * message, 
    const rosidl_message_type_support_t * type_support);

template <typename MessageT>
flexbuffers::Builder& flatten_message(flexbuffers::Builder& builder, const MessageT& message) {
  const rosidl_message_type_support_t* type_support = 
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<MessageT>();
  const void *type_erased_message = static_cast<const void *>(&message);
  return flatten_message(builder, type_erased_message, type_support);
}

namespace detail {

template <typename MessageT>
std::enable_if_t<is_flat_message_v<MessageT>, std::vector<uint8_t>> make_flat_message_image() {
  flexbuffers::Builder builder;
  flatten_message(builder, MessageT::get_prototype_instance());
  return builder.GetBuffer();
}

template <typename MessageT>
std::enable_if_t<is_flat_message_v<MessageT>, const std::vector<uint8_t>&>
get_flat_message_image() {
  static std::vector<uint8_t> image = make_flat_message_image<MessageT>();
  return image;
}

template <typename WrapperT>
void *wrap_loaned_message(uint8_t* payload, size_t size, void *type_erased_wraper, bool copy) {   
  try {
    if (type_erased_wraper != nullptr) {
      auto wrapper = static_cast<WrapperT *>(type_erased_wraper);
      if (copy) {
        wrapper->reset(std::vector<uint8_t>(payload, payload + size));
      } else {
        wrapper->reset(std::span<uint8_t>(payload, size));
      }
    } else {
      if (copy) {
        type_erased_wraper = new WrapperT(std::vector<uint8_t>(payload, payload + size));
      } else {
        type_erased_wraper = new WrapperT(std::span<uint8_t>(payload, size));
      }
    }
    return type_erased_wraper;
  } catch (...) {
    return nullptr;
  }
}

template <typename WrapperT>
uint8_t *unwrap_loaned_message(void *type_erased_wrapper, size_t * size, bool keep) {
  try {
    WrapperT * wrapper = static_cast<WrapperT *>(type_erased_wrapper);
    std::span<uint8_t> payload = wrapper->payload();
    if (size) {
      *size = payload.size();
    }
    if (!keep && !wrapper->owning()) {
      delete wrapper;
    }
    return payload.data();
  } catch (...) {
    return nullptr;
  }
}

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };

template <typename MessageT>
struct FlatImpl {
public:
  FlatImpl() = default;
  FlatImpl(flexbuffers::Reference ref) : view_(ref) {}

  template <typename T>
  FlatView<T>& member(const std::string& name) {
    return view_.template member<T>(name);
  }
protected:
  FlatView<MessageT> view_;
};

}  // namespace detail

template <typename MessageT, typename DerivedT = void>
struct Flat : detail::FlatImpl<MessageT>, detail::FlatStruct<MessageT, Flat<MessageT, DerivedT>> {
public:
  using ProtoT = MessageT;

  template <typename T = DerivedT, std::enable_if_t<std::is_void_v<T>, bool> = true>
  Flat()
  {
  }

  template <typename T = DerivedT, std::enable_if_t<!std::is_void_v<T>, bool> = true>
  Flat() : Flat(detail::get_flat_message_image<DerivedT>())
  {
  }

  template <typename DerivedU>
  Flat(const Flat<MessageT, DerivedU>& other) : detail::FlatImpl<MessageT>() {
    std::visit(detail::overloaded{
      [this](const std::span<uint8_t>& buffer) { 
        reset(std::vector<uint8_t>(buffer.begin(), buffer.end()));
      },
      [this](const std::vector<uint8_t>& buffer) { reset(buffer); },
      [this](const std::monostate&) { reset(); }
    }, other.buffer_);
  }

  Flat(const Flat<MessageT, DerivedT>& other) : detail::FlatImpl<MessageT>() {
    std::visit(detail::overloaded{
      [this](const std::span<uint8_t>& buffer) { 
        reset(std::vector<uint8_t>(buffer.begin(), buffer.end()));
      },
      [this](const std::vector<uint8_t>& buffer) { reset(buffer); },
      [this](const std::monostate&) { reset(); }
    }, other.buffer_);
  }

  template <typename DerivedU>
  Flat<MessageT, DerivedT>& operator=(const Flat<MessageT, DerivedU>& other) {
    std::visit(detail::overloaded{
      [this](const std::span<uint8_t>& buffer) { 
        reset(std::vector<uint8_t>(buffer.begin(), buffer.end()));
      },
      [this](const std::vector<uint8_t>& buffer) { reset(buffer); },
      [this](const std::monostate&) { reset(); }
    }, other.buffer_);
    return *this;
  }

  Flat<MessageT, DerivedT>& operator=(const Flat<MessageT, DerivedT>& other) {
    std::visit(detail::overloaded{
      [this](const std::span<uint8_t>& buffer) { 
        reset(std::vector<uint8_t>(buffer.begin(), buffer.end()));
      },
      [this](const std::vector<uint8_t>& buffer) { reset(buffer); },
      [this](const std::monostate&) { reset(); }
    }, other.buffer_);
    return *this;
  }

private:
  friend void *detail::wrap_loaned_message<Flat<MessageT, DerivedT>>(uint8_t*, size_t, void *, bool);
  friend uint8_t *detail::unwrap_loaned_message<Flat<MessageT, DerivedT>>(void *, size_t *, bool);

  explicit Flat(std::vector<uint8_t> buffer)
    : detail::FlatImpl<MessageT>(flexbuffers::GetRoot(buffer.data(), buffer.size())), buffer_(std::move(buffer))
  {
  }

  explicit Flat(std::span<uint8_t> buffer) 
    : detail::FlatImpl<MessageT>(flexbuffers::GetRoot(buffer.data(), buffer.size())), buffer_(std::move(buffer))
  {
  }

  bool owning() const { return std::holds_alternative<std::vector<uint8_t>>(buffer_); }

  std::span<uint8_t> payload() {
    return std::visit(detail::overloaded{ 
      [](auto& buffer) { return std::span<uint8_t>(buffer.data(), buffer.size()); },
      [](std::monostate&) { return std::span<uint8_t>(); }
    }, buffer_);
  }

  void reset() {
    this->view_.reset();
    buffer_ = std::monostate{}; 
  }

  void reset(std::span<uint8_t> buffer) {
    this->view_.reset(flexbuffers::GetRoot(buffer.data(), buffer.size())); 
    buffer_ = std::move(buffer);
  }

  void reset(std::vector<uint8_t> buffer) {
    this->view_.reset(flexbuffers::GetRoot(buffer.data(), buffer.size())); 
    buffer_ = std::move(buffer);
  }

  std::variant<std::monostate, std::vector<uint8_t>, std::span<uint8_t>> buffer_;
};

template <typename MessageT>
using as_flat_t = std::conditional_t<!is_flat_v<MessageT>, Flat<MessageT>, MessageT>;

namespace detail {

template <typename MessageT>
std::enable_if_t<is_flat_v<MessageT>, rosidl_message_type_support_t> make_flat_message_type_support() 
{
  rosidl_message_type_support_t flat_message_type_support;
  flat_message_type_support.typesupport_identifier = typesupport_identifier;
  auto type_support = std::make_unique<flat_message_type_support_t>();
  if constexpr (is_flat_message_v<MessageT>) {
    const std::vector<uint8_t>& message_image = get_flat_message_image<MessageT>(); 
    type_support->message_size = message_image.size();
    type_support->message_image = message_image.data();
  }
  type_support->wrap_message = &wrap_loaned_message<MessageT>;
  type_support->unwrap_message = &unwrap_loaned_message<MessageT>;
  const rosidl_message_type_support_t& proto_message_type_support =
      rclcpp::get_message_type_support_handle<typename MessageT::ProtoT>();
  flat_message_type_support.data = type_support.release();
  flat_message_type_support.func = get_message_typesupport_handle_function;
  flat_message_type_support.get_type_hash_func = proto_message_type_support.get_type_hash_func;
  flat_message_type_support.get_type_description_func = proto_message_type_support.get_type_description_func;
  flat_message_type_support.get_type_description_sources_func = proto_message_type_support.get_type_description_sources_func;
  return flat_message_type_support;
}

}  // namespace detail

template <typename MessageT>
std::enable_if_t<is_flat_v<MessageT>, const rosidl_message_type_support_t &> get_flat_message_type_support() {
  static rosidl_message_type_support_t type_support = detail::make_flat_message_type_support<MessageT>(); 
  return type_support;
}

}  // namespace flatros2

template<typename MessageT, typename DerivedT>
struct rclcpp::is_ros_compatible_type<flatros2::Flat<MessageT, DerivedT>> : std::true_type {};

template<typename MessageT, typename DerivedT>
struct rosidl_generator_traits::is_message<flatros2::Flat<MessageT, DerivedT>> : std::true_type {};

#define DECLARE_FLAT_PROTOTYPE(Name) \
  template<> \
  const rosidl_message_type_support_t * \
  rosidl_typesupport_cpp::get_message_type_support_handle<flatros2::Flat<Name>>() { \
    return &(flatros2::get_flat_message_type_support<flatros2::Flat<Name>>()); \
  } \
  DECLARE_FLAT_STRUCTURE(Name)

#define DECLARE_FLAT_MESSAGE(Name, Prototype) \
  struct Name : flatros2::Flat<Name, Prototype> { \
    public: \
      static Prototype get_prototype_instance() { \
        Prototype message; \
        build_message_prototype(message); \
        return message; \
      } \
    private: \
      static void build_prototype_instance(Prototype& prototype); \
  }; \
  template<> \
  const rosidl_message_type_support_t * \
  rosidl_typesupport_cpp::get_message_type_support_handle<Name>() { \
    return &(flatros2::get_flat_message_type_support<Name>()); \
  } \
  void Name::build_prototype_instance([[maybe_unused]] Prototype& prototype)

#endif // FLAT_ROS2_MESSAGES_HPP
