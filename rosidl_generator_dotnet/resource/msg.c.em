@{
from rosidl_generator_dotnet import msg_type_to_c
from rosidl_generator_dotnet import get_builtin_dotnet_type
from rosidl_generator_dotnet import get_idl_type

from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamespacedType

from rosidl_cmake import convert_camel_case_to_lower_case_underscore

type_name = message.structure.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(message.structure.namespaced_type.namespaces), type_name)
header_filename = "{0}/rcldotnet_{1}.h".format('/'.join(message.structure.namespaced_type.namespaces), convert_camel_case_to_lower_case_underscore(type_name))
}@

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

#include <@('/'.join(message.structure.namespaced_type.namespaces))/@(convert_camel_case_to_lower_case_underscore(type_name)).h>
#include "rosidl_generator_c/message_type_support_struct.h"

#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

#include <rosidl_generator_c/primitives_sequence.h>
#include <rosidl_generator_c/primitives_sequence_functions.h>


#include "@(header_filename)"

void * @(msg_typename)__create_native_message() {
   @(msg_typename) * ros_message = @(msg_typename)__create();
   return ros_message;
}

const void * @(msg_typename)__get_typesupport() {
  const void * ptr = ROSIDL_GET_MSG_TYPE_SUPPORT(@(package_name), msg, @(type_name));
  return ptr;
}

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array)]@
void * @(msg_typename)_get_field_@(member.name)_message(void *message_handle, int index) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return &(ros_message->@(member.name)[index]);
}

void * @(msg_typename)_init_field_@(member.name)_message(void *message_handle, int size) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[        if isinstance(member.type.value_type, Array)]@
//  TODO: Array array fields are not supported
@[        elif isinstance(member.type.value_type, AbstractSequence)]@ 
//  TODO: AbstractSequence array fields are not supported
@[        elif isinstance(member.type.value_type, AbstractWString)]@ 
//  TODO: AbstractWString array fields are not supported
@[        elif isinstance(member.type.value_type, BasicType) or isinstance(member.type, AbstractString)]@
  rosidl_generator_c__@(get_idl_type(member.type.value_type.typename))__Sequence__init(&(ros_message->@(member.name)), size);
@[        elif isinstance(member.type.value_type, AbstractGenericString)]@
// TODO: AbstractGenericString array fields are not supported
@[        else]@
  @(member.type.value_type.namespaces[0])__@(member.type.value_type.namespaces[1])__@(member.type.value_type.name)__Sequence__init(&(ros_message->@(member.name)), size);
@[        end if]@ 
}

int @(msg_typename)_getsize_array_field_@(member.name)_message(void *message_handle)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  int size = 0;

@[    if isinstance(member.type, Array)]@
  size = @(member.type.size);
@[    end if]@

  return size;
}

@[    elif isinstance(member.type, AbstractSequence)]@
// TODO: Sequence types are not supported
@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
@(msg_type_to_c(member.type)) @(msg_typename)__read_field_@(member.name)(void * message_handle) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[        if isinstance(member.type, AbstractGenericString)]@
  return ros_message->@(member.name).data;
@[        else]@
  return ros_message->@(member.name);
@[        end if]@
}
void @(msg_typename)__write_field_@(member.name)(void * message_handle, @(msg_type_to_c(member.type)) value) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[        if isinstance(member.type, AbstractGenericString)]@
  rosidl_generator_c__String__assign(
    &ros_message->@(member.name), value);
@[        else]@
  ros_message->@(member.name) = value;
@[        end if]@
}
@[    else]@
void * @(msg_typename)__get_field_@(member.name)_HANDLE(void * message_handle) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return &(ros_message->@(member.name));
}
@[    end if]@
@[end for]@

void @(msg_typename)__destroy_native_message(void * raw_ros_message) {
  @(msg_typename) * ros_message = raw_ros_message;
  @(msg_typename)__destroy(ros_message);
}
