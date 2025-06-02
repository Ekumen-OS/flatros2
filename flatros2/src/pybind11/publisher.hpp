#ifndef FLAT_ROS2_PYBIND11_PUBLISHER_HPP
#define FLAT_ROS2_PYBIND11_PUBLISHER_HPP

#include <cstddef>
#include <string>

#include <rcl/publisher.h>

#include <pybind11/pybind11.h>

#include "destroyable.hpp"

namespace py = pybind11;

namespace flatros2 {
namespace pybind11 {

class FlatPublisherImpl : public Destroyable {
  public:
    explicit FlatPublisherImpl(
      py::object pynode_handle,
      py::object pymessage_type,
      const std::string &topic_name,
      const py::object pyqos_profile);

    ~FlatPublisherImpl() override;

    std::string get_topic_name() const;

    py::object borrow_loaned_message();

    void publish_loaned_message(py::object pymessage);

    void return_loaned_message(py::object pymessage);

  private:
    void destroy() override;

    py::object pynode_handle_;
    py::object pymessage_type_;
    rcl_publisher_t rcl_publisher_;
};

} // namespace pybind11
} // namespace flatros2

#endif  // FLAT_ROS2_PYBIND11_PUBLISHER_HPP
