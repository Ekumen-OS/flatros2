#include <pybind11/pybind11.h>

#include <memory>

#include "destroyable.hpp"
#include "publisher.hpp"
#include "subscription.hpp"
#include "typesupport.hpp"

namespace py = pybind11;

using namespace flatros2::pybind11;

PYBIND11_MODULE(flatros2_pybindings, m) {
  m.doc() = "ROS 2 Python client library.";

  py::class_<Destroyable, std::shared_ptr<Destroyable>>(m, "Destroyable")
    .def("__enter__", &Destroyable::enter)
    .def("__exit__", &Destroyable::exit)
    .def("destroy_when_not_in_use", &Destroyable::destroy_when_not_in_use);

  py::class_<FlatPublisherImpl, Destroyable, std::shared_ptr<FlatPublisherImpl>>(m, "FlatPublisherImpl")
    .def(py::init<py::object, py::object, const std::string&, py::object>())
    .def("get_topic_name", &FlatPublisherImpl::get_topic_name)
    .def("borrow_loaned_message",
         &FlatPublisherImpl::borrow_loaned_message,
         py::return_value_policy::reference_internal)
    .def("return_loaned_message", &FlatPublisherImpl::return_loaned_message)
    .def("publish_loaned_message", &FlatPublisherImpl::publish_loaned_message);

  py::class_<FlatSubscriptionImpl, Destroyable, std::shared_ptr<FlatSubscriptionImpl>>(m, "FlatSubscriptionImpl")
    .def(py::init<py::object, py::object, const std::string&, py::object>())
    .def("take_loaned_message", &FlatSubscriptionImpl::take_loaned_message,
         py::return_value_policy::reference_internal)
    .def("return_loaned_message", &FlatSubscriptionImpl::return_loaned_message)
    .def("get_topic_name", &FlatSubscriptionImpl::get_topic_name)
    .def("add_to_waitset", &FlatSubscriptionImpl::add_to_waitset)
    .def("is_ready", &FlatSubscriptionImpl::is_ready);

  m.def("get_flat_message_image", &get_flat_message_image);
  m.def("make_flat_message_type_support", &make_flat_message_type_support);
}
