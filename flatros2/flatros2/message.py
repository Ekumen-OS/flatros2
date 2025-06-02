import re
from typing import Final

import numpy as np
import flatbuffers.flexbuffers as flexbuffers

from rosidl_runtime_py.utilities import get_message
from rosidl_parser.definition import BOOLEAN_TYPE
from rosidl_parser.definition import OCTET_TYPE
from rosidl_parser.definition import CHARACTER_TYPES
from rosidl_parser.definition import FLOATING_POINT_TYPES
from rosidl_parser.definition import INTEGER_TYPES
from rosidl_parser.definition import SIGNED_INTEGER_TYPES
from rosidl_parser.definition import UNSIGNED_INTEGER_TYPES

from .flatros2_pybindings import get_flat_message_image, make_flat_message_type_support

BYTE_TYPES: Final = ("octet", "uint8")

STRING_TYPES: Final = ("string", "wstring")

SIGNED_INTEGER_BYTE_WIDTHS: Final = {
    "int8": 1,
    "int16": 2,
    "int32": 4,
    "int64": 8,
}

UNSIGNED_INTEGER_BYTE_WIDTHS: Final = {
    "uint8": 1,
    "uint16": 2,
    "uint32": 4,
    "uint64": 8,
}

FLOATING_POINT_BYTE_WIDTHS: Final = {
    "float": 4,
    "double": 8
}

PRIMITIVE_TYPES: Final = (
    (BOOLEAN_TYPE, OCTET_TYPE) + INTEGER_TYPES +
    FLOATING_POINT_TYPES + CHARACTER_TYPES
)

PRIMITIVE_DATA_TYPES: Final = {
    "int8": np.int8,
    "int16": np.int16,
    "int32": np.int32,
    "int64": np.int64,
    "uint8": np.uint8,
    "uint16": np.uint16,
    "uint32": np.uint32,
    "uint64": np.uint64,
    "float": np.float32,
    "double": np.float64,
    "char": np.dtype('c'),
    "wchar": np.dtype('U'),
    "octet": np.byte,
}

ARRAY_TYPE_PATTERN = re.compile(r"(?P<basetype>[/\w]+)\[(?P<size>\d+)\]")
SEQUENCE_TYPE_PATTERN = re.compile(r"sequence<(?P<basetype>[/\w]+)>")


class FlatView(type):

    def _make_boolean_property(name, index):
        def getter(self):
            return self._fields[index].AsBool

        def setter(self, value):
            self._fields[index].MutateBool(value)

        return property(getter, setter, doc=f"{name} field")

    def _make_integer_property(name, index):
        def getter(self):
            return self._fields[index].AsInt

        def setter(self, value):
            self._fields[index].MutateInt(value)

        return property(getter, setter, doc=f"{name} field")

    def _make_floating_point_property(name, index):
        def getter(self):
            return self._fields[index].AsFloat

        def setter(self, value):
            self._fields[index].MutateFloat(value)

        return property(getter, setter, doc=f"{name} field")

    def _make_string_property(name, index):
        def getter(self):
            return bytes(self._fields[index].AsBlob).decode("utf-8")

        def setter(self, value):
            view = self._fields[index].AsBlob
            view[:] = value.encode("utf-8")

        return property(getter, setter, doc=f"{name} field")

    def _make_object_property(name, index, view_type):
        def getter(self):
            return view_type(self._fields[index])

        return property(getter, doc=f"{name} field")

    def _make_blob_property(name, index):
        def getter(self):
            return self._fields[index].AsBlob

        def setter(self, value):
            view = getter(self)
            view[:] = value

        return property(getter, setter, doc=f"{name} field")

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

    def _make_string_array_property(name, index):
        def getter(self):
            return StringArrayView(self._fields[index])

        return property(getter, doc=f"{name} field")

    class ObjectArrayView:

        def __init__(self, ref, view_type):
            self._items = ref.AsVector
            self._view_type = view_type

        def __getitem__(self, index):
            return self._view_type(self._items[index])

        def __len__(self):
            return len(self._items)

    def _make_object_array_property(name, index, view_type):
        def getter(self):
            return ObjectArrayView(self._fields[index], view_type)

        return property(getter, doc=f"{name} field")

    def __new__(cls, prototype_class):
        def __init__(self, ref):
            self._fields = ref.AsVector

        attributes = {"__init__": __init__}
        fields = prototype_class.get_fields_and_field_types()
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
        name = f"Flat{prototype_class.__name__}View"
        return super().__new__(cls, name, bases, attributes)


def flatten_message(builder, message):
    fields = message.__class__.get_fields_and_field_types()
    with builder.Vector():
        for i, (name, typename) in enumerate(fields.items()):
            value = getattr(message, name)
            if typename == BOOLEAN_TYPE:
                builder.Bool(value)
            elif typename == OCTET_TYPE:
                builder.Int(value)
            elif typename in SIGNED_INTEGER_TYPES:
                byte_width = SIGNED_INTEGER_BYTE_WIDTHS[typename]
                builder.Int(value, byte_width)
            elif typename in UNSIGNED_INTEGER_TYPES:
                byte_width = UNSIGNED_INTEGER_BYTE_WIDTHS[typename]
                builder.UInt(value, byte_width)
            elif typename in FLOATING_POINT_TYPES:
                byte_width = FLOATING_POINT_BYTE_WIDTHS[typename]
                builder.Float(value, byte_width)
            elif typename in CHARACTER_TYPES or typename in STRING_TYPES:
                builder.Blob(value.encode("utf-8"))
            elif (m := (
                ARRAY_TYPE_PATTERN.fullmatch(typename) or
                SEQUENCE_TYPE_PATTERN.fullmatch(typename)
            )) is not None:
                if m["basetype"] in PRIMITIVE_TYPES:
                    builder.Blob(np.asarray(value).tobytes())
                elif m["basetype"] in STRING_TYPES:
                    with builder.Vector():
                        for item in value:
                            builder.Blob(item.encode("utf-8"))
                else:
                    with builder.Vector():
                        for item in value:
                            flatten_message(builder, item)
            else:
                flatten_message(builder, value)
    return builder


class Flat(FlatView):

    def __new__(cls, prototype):
        builder = flexbuffers.Builder()
        flatten_message(builder, prototype)
        image = builder.Finish()

        typesupport = make_flat_message_type_support(prototype, bytes(image))

        def __init__(self, *, _buffer=None):
            if _buffer is None:
                _buffer = bytearray(get_flat_message_image(self.__class__))
            super(type(self), self).__init__(flexbuffers.GetRoot(_buffer))
            self._buffer = _buffer

        prototype_class = prototype.__class__
        name = f"Flat{prototype_class.__name__}"
        bases = (FlatView(prototype_class),)
        attributes = {
            "__init__": __init__,
            "_TYPE_SUPPORT": typesupport
        }
        return type.__new__(cls, name, bases, attributes)
