#include "destroyable.hpp"

#include <stdexcept>

namespace flatros2 {
namespace pybind11 {

void Destroyable::enter() {
    if (destroy_called_) {
        throw std::runtime_error("cannot take, destruction already requested");
    }
    ++use_count_;
}

void Destroyable::exit(py::object, py::object, py::object) {
    if (0u == use_count_) {
        throw std::runtime_error("not in use, cannot release");
    }
    if (0u == --use_count_ && destroy_called_) {
        destroy();
    }
}

void Destroyable::destroy_when_not_in_use() {
    if (!destroy_called_) {
        destroy_called_ = true;
        if (0u == use_count_) {
            destroy();
        }
    }
}

}  // namespace pybind11
}  // namespace flatros2
