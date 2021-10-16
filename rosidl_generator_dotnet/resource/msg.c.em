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

from re import sub

type_name = message.structure.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(message.structure.namespaced_type.namespaces), type_name)
header_filename = "{0}/rcldotnet_{1}.h".format('/'.join(message.structure.namespaced_type.namespaces), convert_camel_case_to_lower_case_underscore(type_name))
}@

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

#include <@('/'.join(message.structure.namespaced_type.namespaces))/@(convert_camel_case_to_lower_case_underscore(type_name)).h>
#include "rosidl_runtime_c/message_type_support_struct.h"

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>


#include "@(header_filename)"


@[for member in message.structure.members]@
@[    if isinstance(member.type, AbstractSequence)]@
@[        if isinstance(member.type.value_type, NamespacedType)]@
#include "@('/'.join(member.type.value_type.namespaces))/detail/@(convert_camel_case_to_lower_case_underscore(member.type.value_type.name))__struct.h"
#include "@('/'.join(member.type.value_type.namespaces))/detail/@(convert_camel_case_to_lower_case_underscore(member.type.value_type.name))__functions.h"
#include "@('/'.join(member.type.value_type.namespaces))/detail/@(convert_camel_case_to_lower_case_underscore(member.type.value_type.name))__type_support.h"
@[        end if]@
@[    end if]@
@[end for]@

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
////////////////////////////////////////////////////////
// DOING: Array types support

void * @(msg_typename)__get_field_@(member.name)_message(void *message_handle, int index) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return &(ros_message->@(member.name)[index]);
}

int @(msg_typename)__getsize_array_field_@(member.name)_message()
{
@[        if isinstance(member.type, Array)]@
  return @(member.type.size);
@[        else]@
  return 0;
@[        end if]@
}

@[        if isinstance(member.type.value_type, BasicType)]@
void @(msg_typename)__write_field_@(member.name)(void *message_handle, @(msg_type_to_c(member.type.value_type)) value)
{
  @(msg_type_to_c(member.type.value_type)) * ros_message = (@(msg_type_to_c(member.type.value_type)) *)message_handle;
  *ros_message = value;
}

@(msg_type_to_c(member.type.value_type)) @(msg_typename)__read_field_@(member.name)(void *message_handle)
{
  @(msg_type_to_c(member.type.value_type)) * ros_message = (@(msg_type_to_c(member.type.value_type)) *)message_handle;
  return *ros_message;
}
@[        elif isinstance(member.type.value_type, AbstractString)]@
void @(msg_typename)__write_field_@(member.name)(void *message_handle, @(msg_type_to_c(member.type.value_type)) value)
{
  
}

@(msg_type_to_c(member.type.value_type)) @(msg_typename)__read_field_@(member.name)(void *message_handle)
{
  @(msg_typename) * ros_message = (@(msg_typename)*)message_handle;
  return ros_message->@(member.name)->data;
}
@[        end if]@

////////////////////////////////////////////////////////
@[    elif isinstance(member.type, AbstractSequence)]@
////////////////////////////////////////////////////////

void * @(msg_typename)__get_field_@(member.name)_message(void *message_handle, int index) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return &((ros_message->@(member.name).data)[index]);
}

int @(msg_typename)__getsize_sequence_field_@(member.name)_message(void *message_handle) {
@[        if isinstance(member.type, AbstractSequence)]@
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return (int)ros_message->@(member.name).size;
@[        else]@
  return 0;
@[        end if]@
}

int @(msg_typename)__resize_sequence_field_@(member.name)_message(void *message_handle, int size) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  if (size <= 0) {
    return -1;
  }

  if (size < (int)ros_message->@(member.name).size) {
    ros_message->@(member.name).size = size;
    return 0;
  } 
  else if (size > (int)ros_message->@(member.name).size) {
@[        if isinstance(member.type.value_type, BasicType)]@
    if (ros_message->@(member.name).data) {
      rosidl_runtime_c__@(sub("_t$", "", msg_type_to_c(member.type.value_type)))__Sequence__fini(&ros_message->@(member.name));
    }
    return rosidl_runtime_c__@(sub("_t$", "", msg_type_to_c(member.type.value_type)))__Sequence__init(&ros_message->@(member.name), size) ? 0 : -1;
@[        elif isinstance(member.type.value_type, NamespacedType)]@
    if (ros_message->@(member.name).data) {
      @('%s__%s' % ('__'.join(member.type.value_type.namespaces), member.type.value_type.name))__Sequence__fini(&ros_message->@(member.name));
    }
    return @('%s__%s' % ('__'.join(member.type.value_type.namespaces), member.type.value_type.name))__Sequence__init(&ros_message->@(member.name), size) ? 0 : -1;
@[        else]@
    //TODO: non-primitive sequences not yet supported!
    return -1;
@[        end if]@
  }
  return 0;
}

@[        if isinstance(member.type.value_type, BasicType)]@
void @(msg_typename)__write_field_@(member.name)(void *message_handle, @(msg_type_to_c(member.type.value_type)) value) {
  @(msg_type_to_c(member.type.value_type)) * ros_message = (@(msg_type_to_c(member.type.value_type)) *)message_handle;
  *ros_message = value;
}

@(msg_type_to_c(member.type.value_type)) @(msg_typename)__read_field_@(member.name)(void *message_handle) {
  @(msg_type_to_c(member.type.value_type)) * ros_message = (@(msg_type_to_c(member.type.value_type)) *)message_handle;
  return *ros_message;
}
@[        elif isinstance(member.type.value_type, AbstractString)]@
void @(msg_typename)__write_field_@(member.name)(void *message_handle, @(msg_type_to_c(member.type.value_type)) value) {
  
}

@(msg_type_to_c(member.type.value_type)) @(msg_typename)__read_field_@(member.name)(void *message_handle) {
  @(msg_typename) * ros_message = (@(msg_typename)*)message_handle;
  return ros_message->@(member.name).data;
}
@[        end if]@

////////////////////////////////////////////////////////
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
  rosidl_runtime_c__String__assign(
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
