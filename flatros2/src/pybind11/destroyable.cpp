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
