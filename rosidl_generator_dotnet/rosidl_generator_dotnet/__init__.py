# Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

from collections import defaultdict
import os
import string

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser import parse_message_file
from rosidl_parser import parse_service_file


class Underscorer(string.Formatter):
    def format_field(self, value, spec):
        if spec.endswith('underscore'):
            value = convert_camel_case_to_lower_case_underscore(value)
            spec = spec[:-(len('underscore'))] + 's'
        return super(Underscorer, self).format_field(value, spec)


def generate_dotnet(generator_arguments_file, typesupport_impl,
                    typesupport_impls):
    args = read_generator_arguments(generator_arguments_file)
    typesupport_impls = typesupport_impls.split(';')

    template_dir = args['template_dir']
    type_support_impl_by_filename = {
        '{{}}.ep.{impl}.c'.format(impl=impl): impl
        for impl in typesupport_impls
    }
    mapping_msgs = {
        os.path.join(template_dir, 'msg.cs.em'): ['{0}.cs'],
        os.path.join(template_dir, 'msg.c.em'):
        type_support_impl_by_filename.keys(),
        os.path.join(template_dir, 'msg.h.em'): ['rcldotnet_{0:underscore}.h'],
    }

    mapping_srvs = {os.path.join(template_dir, 'srv.cs.em'): ['%s.cs'], }

    for template_file in mapping_msgs.keys():
        assert os.path.exists(template_file), \
            'Messages template file %s not found' % template_file
    for template_file in mapping_srvs.keys():
        assert os.path.exists(template_file), \
            'Services template file %s not found' % template_file

    functions = {'get_dotnet_type': get_dotnet_type, }
    latest_target_timestamp = get_newest_modification_time(
        args['target_dependencies'])

    modules = defaultdict(list)
    for ros_interface_file in args['ros_interface_files']:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == '.msg':
            spec = parse_message_file(args['package_name'], ros_interface_file)
            mapping = mapping_msgs
            type_name = spec.base_type.type
        elif extension == '.srv':
            spec = parse_service_file(args['package_name'], ros_interface_file)
            mapping = mapping_srvs
            type_name = spec.srv_name
        else:
            continue

        module_name = convert_camel_case_to_lower_case_underscore(type_name)
        modules[subfolder].append((module_name, type_name))
        package_name = args['package_name']
        jni_package_name = package_name.replace('_', '_1')
        for template_file, generated_filenames in mapping.items():
            for generated_filename in generated_filenames:
                data = {
                    'constant_value_to_dotnet': constant_value_to_dotnet,
                    'convert_camel_case_to_lower_case_underscore':
                    convert_camel_case_to_lower_case_underscore,
                    'get_builtin_dotnet_type': get_builtin_dotnet_type,
                    'module_name': module_name,
                    'package_name': package_name,
                    'jni_package_name': jni_package_name,
                    'spec': spec,
                    'subfolder': subfolder,
                    'typesupport_impl':
                    type_support_impl_by_filename.get(generated_filename, ''),
                    'typesupport_impls': typesupport_impls,
                    'type_name': type_name,
                    'primitive_msg_type_to_c': primitive_msg_type_to_c,
                    'get_field_name': get_field_name,
                    'header_name': 'rcldotnet_{}'.format(
                        convert_camel_case_to_lower_case_underscore(
                            module_name)),
                }
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder,
                    Underscorer().format(generated_filename, type_name))
                expand_template(
                    template_file,
                    data,
                    generated_file,
                    minimum_timestamp=latest_target_timestamp)

    return 0


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace("'", "\\'")
    return s


def constant_value_to_dotnet(type_, value):
    assert value is not None

    if type_ == 'bool':
        return 'true' if value else 'false'

    if type_ in [
            'byte',
            'char',
            'int8',
            'uint8',
            'int16',
            'uint16',
            'int32',
            'uint32',
            'int64',
            'uint64',
            'float64',
    ]:
        return str(value)

    if type_ == 'float32':
        return '%sf' % value

    if type_ == 'string':
        return '"%s"' % escape_string(value)

    assert False, "unknown constant type '%s'" % type_


def get_builtin_dotnet_type(type_, use_primitives=True):
    if type_ == 'bool':
        return 'bool' if use_primitives else 'System.Boolean'

    if type_ == 'byte':
        return 'byte' if use_primitives else 'System.Byte'

    if type_ == 'char':
        return 'char' if use_primitives else 'System.Char'

    if type_ == 'float32':
        return 'float' if use_primitives else 'System.Single'

    if type_ == 'float64':
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

    if type_ == 'string':
        return 'System.String'

    assert False, "unknown type '%s'" % type_


def get_dotnet_type(type_, use_primitives=True):
    if not type_.is_primitive_type():
        return type_.pkg_name + ".msg." + type_.type

    return get_builtin_dotnet_type(type_.type, use_primitives=use_primitives)


MSG_TYPE_TO_C = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'char',
    'float32': 'float',
    'float64': 'double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': "const char *",
}


def primitive_msg_type_to_c(type_):
    return MSG_TYPE_TO_C[type_]


def msg_type_to_c(type_, name_):
    """
    Convert a message type into the C declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, char *

    @param type_: The message type
    @type type_: rosidl_parser.Type
    @param type_: The field name
    @type type_: str
    """
    c_type = None
    if type_.is_primitive_type():
        c_type = MSG_TYPE_TO_C[type_.type]
    else:
        c_type = '%s__msg__%s' % (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None or type_.is_upper_bound:
            # Dynamic sized array
            if type_.is_primitive_type() and type_.type != 'string':
                c_type = 'rosidl_generator_c__%s' % type_.type
            return '%s__Array %s' % (c_type, name_)
        else:
            # Static sized array (field specific)
            return '%s %s[%d]' % \
                (c_type, name_, type_.array_size)
    else:
        return '%s %s' % (c_type, name_)


def upperfirst(s):
    return s[0].capitalize() + s[1:]


def get_field_name(type_name, field_name):
    if upperfirst(field_name) == type_name:
        return "{0}_".format(type_name)
    else:
        return upperfirst(field_name)
