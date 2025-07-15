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

import re
import numpy as np
from typing import Final

from rosidl_parser.definition import BOOLEAN_TYPE
from rosidl_parser.definition import OCTET_TYPE
from rosidl_parser.definition import CHARACTER_TYPES
from rosidl_parser.definition import FLOATING_POINT_TYPES
from rosidl_parser.definition import INTEGER_TYPES
from rosidl_parser.definition import SIGNED_INTEGER_TYPES
from rosidl_parser.definition import UNSIGNED_INTEGER_TYPES

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
    "wchar": np.dtype('U1'),
    "octet": np.byte,
}

ARRAY_TYPE_PATTERN = re.compile(r"(?P<basetype>[/\w]+)\[(?P<size>\d+)\]")
SEQUENCE_TYPE_PATTERN = re.compile(r"sequence<(?P<basetype>[/\w]+)>")