// generated from rosidl_generator_dotnet/resource/idl.h.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>.h files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore

header_guard_parts = ["RCLDONTET", package_name] + list(interface_path.parents[0].parts) + [
    convert_camel_case_to_lower_case_underscore(interface_path.stem)]
header_guard = '__'.join([x.upper() for x in header_guard_parts]) + '__H'

}@
#ifndef @(header_guard)
#define @(header_guard)

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
    'msg.h.em',
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
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=service.request_message)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=service.response_message)
}@

@{
TEMPLATE(
    'srv.h.em',
    package_name=package_name, interface_path=interface_path, service=service)
}@
@[end for]@
@
@
@#######################################################################
@# Handle actions
@#######################################################################
@
@{
from rosidl_parser.definition import Action
}@
@[for action in content.get_elements_of_type(Action)]@
@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.goal)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.result)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.feedback)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.send_goal_service.request_message)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.send_goal_service.response_message)
}@

@{
TEMPLATE(
    'srv.h.em',
    package_name=package_name, interface_path=interface_path, service=action.send_goal_service)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.get_result_service.request_message)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.get_result_service.response_message)
}@

@{
TEMPLATE(
    'srv.h.em',
    package_name=package_name, interface_path=interface_path, service=action.get_result_service)
}@

@{
TEMPLATE(
    'msg.h.em',
    package_name=package_name, interface_path=interface_path, message=action.feedback_message)
}@

@{
TEMPLATE(
    'action.h.em',
    package_name=package_name, interface_path=interface_path, action=action)
}@
@[end for]@
@

#endif // @(header_guard)
