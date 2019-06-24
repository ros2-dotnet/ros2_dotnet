# Copyright 2014-2016 Open Source Robotics Foundation, Inc.
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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_generator_c import primitive_msg_type_to_c
from rosidl_parser import parse_message_file
from rosidl_parser import parse_service_file

import logging


def generate_cs(generator_arguments_file, typesupport_impls):
    args = read_generator_arguments(generator_arguments_file)

    template_dir = args['template_dir']
    type_support_impl_by_filename = {
        '_%s_s.ep.{0}.c'.format(impl): impl for impl in typesupport_impls
    }

    logging.info("Generating C# interface code")

    mapping_msgs = {
        os.path.join(template_dir, '_msg.cs.em'): ['_%s.cs'],
        os.path.join(template_dir, '_msg_support.c.em'): ['_%s_s.c'],
    }
    mapping_msg_pkg_extension = {
        os.path.join(template_dir, '_msg_pkg_typesupport_entry_point.c.em'):
        type_support_impl_by_filename.keys(),
    }
    mapping_srvs = {
        os.path.join(template_dir, '_srv.cs.em'): ['_%s.cs'],
    }

    for template_file in mapping_msgs.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file
    for template_file in mapping_msg_pkg_extension.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file
    for template_file in mapping_srvs.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file

    functions = {
        'convert_camel_case_to_lower_case_underscore': convert_camel_case_to_lower_case_underscore,
        'primitive_msg_type_to_c': primitive_msg_type_to_c,
        'get_dotnet_type': get_dotnet_type,
    }

    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])

    modules = defaultdict(list)
    message_specs = []
    service_specs = []
    for ros_interface_file in args['ros_interface_files']:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == '.msg':
            spec = parse_message_file(args['package_name'], ros_interface_file)
            message_specs.append((spec, subfolder))
            mapping = mapping_msgs
            type_name = spec.base_type.type
        elif extension == '.srv':
            spec = parse_service_file(args['package_name'], ros_interface_file)
            service_specs.append((spec, subfolder))
            mapping = mapping_srvs
            type_name = spec.srv_name
        else:
            continue

        module_name = convert_camel_case_to_lower_case_underscore(type_name)
        modules[subfolder].append((module_name, type_name))
        for template_file, generated_filenames in mapping.items():
            for generated_filename in generated_filenames:
                data = {
                    'module_name': module_name,
                    'package_name': args['package_name'],
                    'spec': spec, 'subfolder': subfolder,
                }
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename % module_name)
                expand_template(
                    template_file, data, generated_file,
                    minimum_timestamp=latest_target_timestamp)

    for template_file, generated_filenames in mapping_msg_pkg_extension.items():
        for generated_filename in generated_filenames:
            data = {
                'package_name': args['package_name'],
                'message_specs': message_specs,
                'service_specs': service_specs,
                'typesupport_impl': type_support_impl_by_filename.get(generated_filename, ''),
            }
            data.update(functions)
            generated_file = os.path.join(
                args['output_dir'], generated_filename % args['package_name'])
            expand_template(
                template_file, data, generated_file,
                minimum_timestamp=latest_target_timestamp)

    return 0


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
