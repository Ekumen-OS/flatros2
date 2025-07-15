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

#include <codecvt>
#include <locale>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include "flatros2/message.hpp"

namespace flatros2 {
namespace detail {

flexbuffers::Builder& 
flatten_message(flexbuffers::Builder& builder, const void * message, const rosidl_message_type_support_t * type_support)
{
    static std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> utf8_codec;
    static std::wstring_convert<std::codecvt_utf16<char32_t, 0x10FFFF, std::little_endian>, char32_t> utf32le_codec;

    const rosidl_message_type_support_t * introspection_type_support = 
        get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    const auto *message_description = 
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_type_support->data);
    builder.Vector([&] {
        for (size_t i = 0; i < message_description->member_count_; ++i) {
            const rosidl_typesupport_introspection_cpp::MessageMember& member = message_description->members_[i];
            const void *member_value = static_cast<const uint8_t *>(message) + member.offset_;
            if (member.is_array_) {
                switch (member.type_id_) {
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
                        static_assert(sizeof(bool) == sizeof(uint8_t));
                        std::vector<uint8_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < member.size_function(member_value); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(uint8_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET: {
                        static_assert(sizeof(unsigned char) == sizeof(uint8_t));
                        std::vector<uint8_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(uint8_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
                        std::vector<uint8_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(uint8_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
                        std::vector<uint16_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(uint16_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
                        std::vector<uint32_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(uint32_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
                        std::vector<uint64_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(uint64_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
                        std::vector<int8_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(int8_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
                        std::vector<int16_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(int16_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
                        std::vector<int32_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(int32_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
                        std::vector<int64_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(int64_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32: {
                        std::vector<float> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(float));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64: {
                        std::vector<double> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(double));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
                        std::vector<unsigned char> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        builder.Blob(storage.data(), storage.size() * sizeof(unsigned char));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR: {
                        std::vector<char16_t> storage(member.size_function(member_value));
                        for (size_t j = 0; j < storage.size(); ++j) {
                            member.fetch_function(member_value, j, &storage[j]);
                        }
                        const std::u32string encoded_string = utf32le_codec.from_bytes(
                            reinterpret_cast<const char *>(storage.data()),
                            reinterpret_cast<const char *>(storage.data() + storage.size()));
                        const auto *buffer = reinterpret_cast<const uint8_t *>(storage.data());
                        builder.Blob(buffer, encoded_string.size() * sizeof(char32_t));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
                        builder.Vector([&] {
                            for (size_t j = 0; j < member.size_function(member_value); ++j) {
                                const std::string& item_value = 
                                    *reinterpret_cast<const std::string *>(
                                        member.get_const_function(member_value, j));
                                builder.Blob(item_value.data(), item_value.size());
                            }
                        });
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
                        builder.Vector([&] {
                            for (size_t j = 0; j < member.size_function(member_value); ++j) {
                                const auto *item_value = 
                                    reinterpret_cast<const std::u16string *>(
                                        member.get_const_function(member_value, j));
                                const std::string bytes = utf8_codec.to_bytes(*item_value);
                                builder.Blob(bytes.data(), bytes.size());
                            }
                        });
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                        builder.Vector([&] {
                            for (size_t j = 0; j < member.size_function(member_value); ++j) {
                                detail::flatten_message(builder, member.get_const_function(member_value, j), member.members_);
                            }
                        });
                        break;
                    }
                    default:
                        throw std::runtime_error("unknown message member type");
                }
            } else {
                switch (member.type_id_) {
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
                        builder.Bool(*reinterpret_cast<const bool *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET: {
                        builder.UInt(static_cast<uint8_t>(
                            *reinterpret_cast<const unsigned char *>(member_value)));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
                        builder.UInt(*reinterpret_cast<const uint8_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
                        builder.UInt(*reinterpret_cast<const uint16_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
                        builder.UInt(*reinterpret_cast<const uint32_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
                        builder.UInt(*reinterpret_cast<const uint64_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
                        builder.Int(*reinterpret_cast<const int8_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
                        builder.Int(*reinterpret_cast<const int16_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
                        builder.Int(*reinterpret_cast<const int32_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
                        builder.Int(*reinterpret_cast<const int64_t *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32: {
                        builder.Float(*reinterpret_cast<const float *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64: {
                        builder.Double(*reinterpret_cast<const double *>(member_value));
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
                        builder.Blob(reinterpret_cast<const unsigned char *>(member_value), 1);
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR: {
                        const std::string bytes = utf8_codec.to_bytes(
                            *reinterpret_cast<const char16_t *>(member_value));
                        builder.Blob(bytes.data(), bytes.size());
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
                        const std::string& bytes = *reinterpret_cast<const std::string *>(member_value);
                        builder.Blob(bytes.data(), bytes.size());
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
                        const std::string bytes = utf8_codec.to_bytes(
                            *reinterpret_cast<const std::u16string *>(member_value));
                        builder.Blob(bytes.data(), bytes.size());
                        break;
                    }
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                        detail::flatten_message(builder, member_value, member.members_);
                        break;
                    }
                    default: {
                        throw std::runtime_error("unknown message member type");
                    }
                }
            }
        } 
    });
    return builder;
}

} // namespace detail

} // namespace flatros2
