#ifndef FLAT_ROS2_PYBIND11_DESTROYABLE_HPP
#define FLAT_ROS2_PYBIND11_DESTROYABLE_HPP

#include <cstddef>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace flatros2 {
namespace pybind11 {

class Destroyable {
  public:
    Destroyable() = default;

    virtual ~Destroyable() = default;

    virtual void enter();

    virtual void exit(py::object pytype, py::object pyvalue, py::object pytraceback);

    void destroy_when_not_in_use();

  private:
    virtual void destroy() = 0;

    bool destroy_called_{false};
    size_t use_count_{0u};
};

}  // namespace pybind11
}  // namespace flatros2

#endif // FLAT_ROS2_PYBIND11_DESTROYABLE_HPP
