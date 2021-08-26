// generated from rosidl_generator_dotnet/resource/idl.cs.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>.cs files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;

using ROS2.Interfaces;
using ROS2.Utils;

@#######################################################################
@# Handle messages
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg.cs.em',
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
    'msg.cs.em',
    package_name=package_name, interface_path=interface_path, message=service.request_message)
}@

@{
TEMPLATE(
    'msg.cs.em',
    package_name=package_name, interface_path=interface_path, message=service.response_message)
}@

@{
TEMPLATE(
    'srv.cs.em',
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