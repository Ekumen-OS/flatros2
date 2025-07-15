# Copyright 2025 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import flatbuffers.flexbuffers as flexbuffers

from rosidl_runtime_py.utilities import get_message

from flatros2.primitives import *  # noqa


class FlatView(type):

    @staticmethod
    def _make_boolean_property(name, index):
        def getter(self):
            return self._fields[index].AsBool

        def setter(self, value):
            self._fields[index].MutateBool(value)

        return property(getter, setter, doc=f"{name} field")

    @staticmethod
    def _make_octet_property(name, index):
        def getter(self):
            return self._fields[index].AsInt.to_bytes(1)

        def setter(self, value):
            self._fields[index].MutateInt(int.from_bytes(value))

        return property(getter, setter, doc=f"{name} field")

    @staticmethod
    def _make_integer_property(name, index):
        def getter(self):
            return self._fields[index].AsInt

        def setter(self, value):
            self._fields[index].MutateInt(value)

        return property(getter, setter, doc=f"{name} field")

    @staticmethod
    def _make_floating_point_property(name, index):
        def getter(self):
            return self._fields[index].AsFloat

        def setter(self, value):
            self._fields[index].MutateFloat(value)

        return property(getter, setter, doc=f"{name} field")

    @staticmethod
    def _make_string_property(name, index):
        def getter(self):
            return bytes(self._fields[index].AsBlob).decode("utf-8")

        def setter(self, value):
            view = self._fields[index].AsBlob
            view[:] = value.encode("utf-8")

        return property(getter, setter, doc=f"{name} field")

    @staticmethod
    def _make_object_property(name, index, view_type):
        def getter(self):
            return view_type(self._fields[index])

        return property(getter, doc=f"{name} field")

    @staticmethod
    def _make_blob_property(name, index):
        def getter(self):
            return self._fields[index].AsBlob

        def setter(self, value):
            view = getter(self)
            view[:] = value

        return property(getter, setter, doc=f"{name} field")

    @staticmethod
    def _make_array_property(name, index, data_type):
        def getter(self):
            blob = self._fields[index].AsBlob
            return np.frombuffer(blob, data_type)

        def setter(self, value):
            array = getter(self)
            array[:] = value

        return property(getter, setter, doc=f"{name} field")

    class StringArrayView:

        def __init__(self, ref):
            self._items = ref.AsVector

        def __getitem__(self, index):
            return bytes(self._fields[index].AsBlob).decode("utf-8")

        def __setitem__(self, index, value):
            view = self._fields[index].AsBlob
            view[:] = value.encode("utf-8")

        def __len__(self):
            return len(self._items)
    
    @staticmethod
    def _make_string_array_property(name, index):
        def getter(self):
            return FlatView.StringArrayView(self._fields[index])

        return property(getter, doc=f"{name} field")

    class ObjectArrayView:

        def __init__(self, ref, view_type):
            self._items = ref.AsVector
            self._view_type = view_type

        def __getitem__(self, index):
            return self._view_type(self._items[index])

        def __len__(self):
            return len(self._items)

    @staticmethod
    def _make_object_array_property(name, index, view_type):
        def getter(self):
            return FlatView.ObjectArrayView(self._fields[index], view_type)

        return property(getter, doc=f"{name} field")

    def __new__(cls, prototype: type):
        def __init__(self, ref):
            self._fields = ref.AsVector

        attributes = {"__init__": __init__}
        fields = prototype.get_fields_and_field_types()
        for i, (name, typename) in enumerate(fields.items()):
            if typename == BOOLEAN_TYPE:
                attributes[name] = cls._make_boolean_property(name, i)
            elif typename == OCTET_TYPE:
                attributes[name] = cls._make_octet_property(name, i)
            elif typename in INTEGER_TYPES:
                attributes[name] = cls._make_integer_property(name, i)
            elif typename in FLOATING_POINT_TYPES:
                attributes[name] = cls._make_floating_point_property(name, i)
            elif typename in CHARACTER_TYPES or typename in STRING_TYPES:
                attributes[name] = cls._make_string_property(name, i)
            elif (m := (
                ARRAY_TYPE_PATTERN.fullmatch(typename) or
                SEQUENCE_TYPE_PATTERN.fullmatch(typename)
            )) is not None:
                if m["basetype"] in BYTE_TYPES:
                    attributes[name] = cls._make_blob_property(name, i)
                elif m["basetype"] in STRING_TYPES:
                    attributes[name] = cls._make_string_array_property(name, i)
                elif m["basetype"] in PRIMITIVE_TYPES:
                    attributes[name] = cls._make_array_property(
                        name, i, PRIMITIVE_DATA_TYPES[m["basetype"]])
                else:
                    attributes[name] = cls._make_object_array_property(
                        name, i, FlatView(get_message(m["basetype"])))
            else:
                attributes[name] = cls._make_object_property(
                    name, i, FlatView(get_message(typename)))

        bases = (object,)
        name = f"Flat{prototype.__name__}View"
        return super().__new__(cls, name, bases, attributes)