#ifndef FLAT_ROS2_TYPESUPPORT_HPP
#define FLAT_ROS2_TYPESUPPORT_HPP

#include <cstddef>
#include <cstdint>

namespace flatros2 {

typedef struct flat_message_type_support_s {
  uint8_t *message_image;
  size_t message_size;
} flat_message_type_support_t;

inline const char typesupport_identifier[] = "flat";

}  // namespace flatros2

#endif // FLAT_ROS2_TYPESUPPORT_HPP
