# Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
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

import os
import string

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import generate_files
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_generator_c import BASIC_IDL_TYPES_TO_C
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import IdlContent
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import NamespacedType
from rosidl_parser.parser import parse_idl_file

class Underscorer(string.Formatter):
    def format_field(self, value, spec):
        if spec.endswith('underscore'):
            value = convert_camel_case_to_lower_case_underscore(value)
            spec = spec[:-(len('underscore'))] + 's'
        return super(Underscorer, self).format_field(value, spec)


def generate_dotnet(generator_arguments_file, typesupport_impls):
    mapping = {
        'idl.cs.em': '%s.cs',
        'idl.c.em': '%s.c',
        'idl.h.em': 'rcldotnet_%s.h'
    }
    generate_files(generator_arguments_file, mapping)
    return 0


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace("'", "\\'")
    return s


def constant_value_to_dotnet(type_, value):
    assert value is not None

    if isinstance(type_, BasicType):
        if type_.typename == 'boolean':
            return 'true' if value else 'false'

        if type_.typename == 'float':
            return '%sf' % value

        if type_.typename in [
                'double',
                'long double',
                'char',
                'wchar',
                'octet',
                'int8',
                'uint8',
                'int16',
                'uint16',
                'int32',
                'uint32',
                'int64',
                'uint64',
        ]:
            return str(value)

    if isinstance(type_, AbstractGenericString):
        return '"%s"' % escape_string(value)

    assert False, "unknown constant type '%s'" % type_

def get_builtin_dotnet_type(type_, use_primitives=True):
    if type_ == 'boolean':
        return 'bool' if use_primitives else 'System.Boolean'

    if type_ == 'byte':
        return 'byte' if use_primitives else 'System.Byte'

    if type_ == 'char':
        return 'char' if use_primitives else 'System.Char'

    if type_ == 'octet':
        return 'byte' if use_primitives else 'System.Byte'

    if type_ == 'float':
        return 'float' if use_primitives else 'System.Single'

    if type_ == 'double':
        return 'double' if use_primitives else 'System.Double'

    if type_ == 'int8':
        return 'sbyte' if use_primitives else 'System.Sbyte'

    if type_ == 'uint8':
        return 'byte' if use_primitives else 'System.Byte'

    if type_ == 'int16':
        return 'short' if use_primitives else 'System.Int16'

    if type_ == 'uint16':
        return 'ushort' if use_primitives else 'System.UInt16'

    if type_ == 'int32':
        return 'int' if use_primitives else 'System.Int32'

    if type_ == 'uint32':
        return 'uint' if use_primitives else 'System.UInt32'

    if type_ == 'int64':
        return 'long' if use_primitives else 'System.Int64'

    if type_ == 'uint64':
        return 'ulong' if use_primitives else 'System.UInt64'

    assert False, "unknown type '%s'" % type_

def get_dotnet_type(type_, use_primitives=True):
    if isinstance(type_, AbstractGenericString):
        return 'System.String'
    if isinstance(type_, NamespacedType):
        return '.'.join(type_.namespaced_name())

    return get_builtin_dotnet_type(type_.typename, use_primitives=use_primitives)

def msg_type_to_c(type_):
    if isinstance(type_, AbstractString):
        return 'char *'
    if isinstance(type_, AbstractWString):
        assert False, "Unicode strings not supported"
    assert isinstance(type_, BasicType)
    return BASIC_IDL_TYPES_TO_C[type_.typename]

def upperfirst(s):
    return s[0].capitalize() + s[1:]


def get_field_name(type_name, field_name):
    if upperfirst(field_name) == type_name:
        return "{0}_".format(type_name)
    else:
        return upperfirst(field_name)
