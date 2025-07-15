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

from typing import Any

import numpy as np
import flatbuffers.flexbuffers as flexbuffers

from flatros2.primitives import *  # noqa
from flatros2.view import FlatView

from rclpy.type_support import check_for_type_support

def flatten_message(builder: flexbuffers.Builder, message: Any) -> flexbuffers.Builder:
    fields = message.__class__.get_fields_and_field_types()
    with builder.Vector():
        for i, (name, typename) in enumerate(fields.items()):
            value = getattr(message, name)
            if typename == BOOLEAN_TYPE:
                builder.Bool(value)
            elif typename == OCTET_TYPE:
                builder.UInt(value)
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

    @staticmethod
    def _make_message_type_support(prototype):
        from .flatros2_pybindings import make_flat_message_type_support
        if not isinstance(prototype, type):
            builder = flexbuffers.Builder()
            flatten_message(builder, prototype)
            image = builder.Finish()
            check_for_type_support(prototype.__class__)
            return make_flat_message_type_support(prototype.__class__, bytes(image))
        check_for_type_support(prototype)
        return make_flat_message_type_support(prototype)

    def __new__(cls, prototype):
        typesupport = cls._make_message_type_support(prototype)

        def __init__(self, *, _buffer=None):
            if _buffer is None:
                from .flatros2_pybindings import get_flat_message_image
                _buffer = bytearray(get_flat_message_image(self.__class__))
            super(type(self), self).__init__(flexbuffers.GetRoot(_buffer))
            self._buffer = _buffer

        if not isinstance(prototype, type):
            prototype = prototype.__class__
        name = f"Flat{prototype.__name__}"
        bases = (FlatView(prototype),)
        attributes = {
            "__init__": __init__,
            "_TYPE_SUPPORT": typesupport
        }
        return type.__new__(cls, name, bases, attributes)