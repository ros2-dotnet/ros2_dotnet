// generated from rosidl_generator_dotnet/resource/idl.c.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>.c files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore

c_header_parts = [package_name] + list(interface_path.parents[0].parts) + [
    convert_camel_case_to_lower_case_underscore(interface_path.stem) + ".h"]
c_header_filename = '/'.join(c_header_parts)

dotnet_header_parts = [package_name] + list(interface_path.parents[0].parts) + [
    "rcldotnet_" + convert_camel_case_to_lower_case_underscore(interface_path.stem) + ".h"]
dotnet_header_filename = '/'.join(dotnet_header_parts)

}@
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

#include <rcutils/allocator.h>

#include "@(c_header_filename)"
#include "rosidl_runtime_c/message_type_support_struct.h"

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "@(dotnet_header_filename)"

@
@#######################################################################
@# Handle messages
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg.c.em',
    package_name=package_name, interface_path=interface_path, message=message)
}@
@[end for]@
@
@#######################################################################
@# Handle services
@#######################################################################
@
@{
from rosidl_parser.definition import Service
}@
@[for service in content.get_elements_of_type(Service)]@
@{
TEMPLATE(
    'msg.c.em',
    package_name=package_name, interface_path=interface_path, message=service.request_message)
}@

@{
TEMPLATE(
    'msg.c.em',
    package_name=package_name, interface_path=interface_path, message=service.response_message)
}@

@{
TEMPLATE(
    'srv.c.em',
    package_name=package_name, interface_path=interface_path, service=service)
}@
@[end for]@
@
@
@#######################################################################
@# Handle actions
@#######################################################################
@
@#TODO - actions not implemented
@