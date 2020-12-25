# Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

import collections
import enum
import struct


_RESERVED_KEYWORDS = set([
    'False',
    'None',
    'True',
    'and',
    'as',
    'assert',
    'break',
    'class',
    'continue',
    'def',
    'del',
    'elif',
    'else',
    'except',
    'finally',
    'for',
    'from',
    'global',
    'if',
    'import',
    'in',
    'is',
    'lambda',
    'nonlocal',
    'not',
    'or',
    'pass',
    'raise',
    'return',
    'try',
    'while',
    'with',
    'yield',
])


def _escape_python3_identifier(name):
    if name in _RESERVED_KEYWORDS:
        return 'py_' + name
    return name


class ParseError(RuntimeError):
    pass


class Stream:
    def __init__(self, base):
        self._base = base

    def ignore(self, size):
        self._base.read(size)

    def read_varuint(self):
        result = 0
        fk = 1
        value = 0

        while True:
            data = self._base.read(1)
            if len(data) != 1:
                raise EOFError()
            value = data[0]
            result = result + fk * (value & 0x7f)
            fk = fk * 128

            if value < 0x80:
                break

            if result >= 2**64:
                raise ParseError("invalid varuint")

        return result

    def read_string(self):
        size = self.read_varuint()
        return self._base.read(size).decode('utf8')

    def read_bytes(self):
        size = self.read_varuint()
        return self._base.read(size)

    def _read_format(self, fmt, size):
        return struct.unpack(fmt, self._base.read(size))[0]

    def read_f32(self):
        return self._read_format('<f', 4)

    def read_f64(self):
        return self._read_format('<d', 8)

    def read_u8(self):
        return self._read_format('<B', 1)

    def read_u16(self):
        return self._read_format('<H', 2)

    def read_u32(self):
        return self._read_format('<I', 4)

    def read_u64(self):
        return self._read_format('<Q', 8)

    def read_i8(self):
        return self._read_format('<b', 1)

    def read_i16(self):
        return self._read_format('<h', 2)

    def read_i32(self):
        return self._read_format('<i', 4)

    def read_i64(self):
        return self._read_format('<q', 8)


class FinalType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return FinalType()

    def read(self, data_stream):
        raise ParseError("invalid")


class NullType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return NullType()

    def read(self, data_stream):
        return None


class BooleanType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return BooleanType()

    def read(self, data_stream):
        return data_stream.read_u8() != 0


class FixedIntType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return FixedIntType(schema_stream.read_u8())

    def __init__(self, field_size):
        if field_size not in [1, 2, 4, 8]:
            raise ParseError("invalid fixedint size")

        self.field_size = field_size

    def read(self, data_stream):
        if self.field_size == 1:
            return data_stream.read_i8()
        elif self.field_size == 2:
            return data_stream.read_i16()
        elif self.field_size == 4:
            return data_stream.read_i32()
        elif self.field_size == 8:
            return data_stream.read_i64()

        assert False


class FixedUIntType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return FixedUIntType(schema_stream.read_u8())

    def __init__(self, field_size):
        if field_size not in [1, 2, 4, 8]:
            raise ParseError("invalid fixeduint size")

        self.field_size = field_size

    def read(self, data_stream):
        if self.field_size == 1:
            return data_stream.read_u8()
        elif self.field_size == 2:
            return data_stream.read_u16()
        elif self.field_size == 4:
            return data_stream.read_u32()
        elif self.field_size == 8:
            return data_stream.read_u64()

        assert False


class VarintType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return VarintType()

    def read(self, data_stream):
        raise RuntimeError("not implemented")


class VaruintType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return VaruintType()

    def read(self, data_stream):
        return data_stream.read_varuint()


class Float32Type:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return Float32Type()

    def read(self, data_stream):
        return data_stream.read_f32()


class Float64Type:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return Float64Type()

    def read(self, data_stream):
        return data_stream.read_f64()


class BytesType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return BytesType()

    def read(self, data_stream):
        return data_stream.read_bytes()


class StringType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return StringType()

    def read(self, data_stream):
        return data_stream.read_string()


class Field:
    def __init__(self, flags, name, aliases, type_class, default_value):
        self.flags = flags
        self.name = name
        self.aliases = aliases
        self.type_class = type_class
        self.default_value = default_value

    def read(self, data_stream):
        return self.type_class.read(data_stream)


class ObjectType:
    @staticmethod
    def from_binary(schema_stream, name='_', **kwargs):
        object_flags = schema_stream.read_varuint()
        fields = []

        while True:
            flags = schema_stream.read_varuint()
            name = schema_stream.read_string()
            naliases = schema_stream.read_varuint()
            aliases = [schema_stream.read_string() for _ in range(naliases)]
            type_class = Type.from_binary(schema_stream, name=name)
            is_default = schema_stream.read_u8() != 0
            default_value = type_class.read(schema_stream) if is_default else None
            if isinstance(type_class, FinalType):
                break
            fields.append(
                Field(flags, name, aliases, type_class, default_value))

        return ObjectType(flags, fields, name)

    def __init__(self, flags, fields, name):
        self.flags = flags
        self.fields = fields
        self.namedtuple = collections.namedtuple(
            '_', [_escape_python3_identifier(x.name) for x in self.fields])

    def read(self, data_stream):
        return self.namedtuple._make([x.read(data_stream) for x in self.fields])


class EnumType:
    @staticmethod
    def from_binary(schema_stream, name='_', **kwargs):
        type_class = Type.from_binary(schema_stream)
        nvalues = schema_stream.read_varuint()
        items = [
            (type_class.read(schema_stream), schema_stream.read_string())
            for _ in range(nvalues)]
        items = { key : value for (value, key) in items }

        return EnumType(name, type_class, items)

    def __init__(self, name, type_class, items):
        self.type_class = type_class

        class Enum(enum.IntEnum):
            @classmethod
            def _missing_(cls, value):
                return cls._create_pseudo_member_(value)

            @classmethod
            def _create_pseudo_member_(cls, value):
                pseudo_member = cls._value2member_map_.get(value, None)
                if pseudo_member is None:
                    new_member = int.__new__(cls, value)
                    new_member._name_ = str(value)
                    new_member._value_ = value
                    pseudo_member = cls._value2member_map_.setdefault(value, new_member)
                return pseudo_member

        self.enum_class = Enum(name, items)

    def read(self, data_stream):
        return self.enum_class(self.type_class.read(data_stream))


class ArrayType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return ArrayType(Type.from_binary(schema_stream))

    def __init__(self, type_class):
        self.type_class = type_class

    def read(self, data_stream):
        nvalues = data_stream.read_varuint()
        return [self.type_class.read(data_stream) for _ in range(nvalues)]


class FixedArrayType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        size = schema_stream.read_varuint()
        return FixedArrayType(size, Type.from_binary(schema_stream))

    def __init__(self, size, type_class):
        self.size = size
        self.type_class = type_class

    def read(self, data_stream):
        return [self.type_class.read(data_stream) for _ in range(self.size)]


class MapType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        type_class = Type.from_binary(schema_stream)
        return MapType(type_class)

    def __init__(self, type_class):
        self.type_class = type_class

    def read(self, data_stream):
        nitems = data_stream.read_varuint()
        return dict((data_stream.read_string(), self.type_class.read(data_stream))
                    for _ in range(nitems))


class UnionType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        items = []
        while True:
            type_class = Type.from_binary(schema_stream)
            if isinstance(type_class, FinalType):
                break
            items.append(type_class)

        return UnionType(items)

    def __init__(self, items):
        self.items = items

    def read(self, data_stream):
        index = data_stream.read_varuint()
        return self.items[index].read(data_stream)


class TimestampType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return TimestampType()

    def read(self, data_stream):
        us_since_epoch = data_stream.read_i64()
        return us_since_epoch / 1000000.0


class DurationType:
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        return DurationType()

    def read(self, data_stream):
        us = data_stream.read_i64()
        return us / 1000000.0


TYPES = [
    FinalType,      # 0
    NullType,       # 1
    BooleanType,    # 2
    FixedIntType,   # 3
    FixedUIntType,  # 4
    VarintType,     # 5
    VaruintType,    # 6
    Float32Type,    # 7
    Float64Type,    # 8
    BytesType,      # 9
    StringType,     # 10
    None,
    None,
    None,
    None,
    None,
    ObjectType,     # 16
    EnumType,       # 17
    ArrayType,      # 18
    FixedArrayType, # 19
    MapType,        # 20
    UnionType,      # 21
    TimestampType,  # 22
    DurationType,   # 23
]


_TYPES_FROM_BINARY = [x.from_binary if x else None for x in TYPES]


class Type:
    '''Read a telemetry serialized schema'''
    @staticmethod
    def from_binary(schema_stream, **kwargs):
        if not isinstance(schema_stream, Stream):
            schema_stream = Stream(schema_stream)

        type_index = schema_stream.read_varuint()
        try:
            this_type = _TYPES_FROM_BINARY[type_index]
        except IndexError:
            raise RuntimeError("Unknown type: {}".format(type_index))
        return this_type(schema_stream, **kwargs)
