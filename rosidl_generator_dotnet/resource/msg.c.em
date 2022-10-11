@{
from rosidl_generator_c import idl_type_to_c

from rosidl_generator_dotnet import msg_type_to_c

from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import BOOLEAN_TYPE

from rosidl_cmake import convert_camel_case_to_lower_case_underscore

type_name = message.structure.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(message.structure.namespaced_type.namespaces), type_name)
}@
void * @(msg_typename)__create_native_message() {
   @(msg_typename) * ros_message = @(msg_typename)__create();
   return ros_message;
}

const void * @(msg_typename)__get_typesupport() {
  const void * ptr = ROSIDL_GET_MSG_TYPE_SUPPORT(@(', '.join(message.structure.namespaced_type.namespaced_name())));
  return ptr;
}

@[for member in message.structure.members]@
@[    if isinstance(member.type, Array) or isinstance(member.type, AbstractSequence)]@
void * @(msg_typename)__get_field_@(member.name)_message(void *message_handle, int32_t index) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[        if isinstance(member.type, Array)]@
  return &(ros_message->@(member.name)[index]);
@[        elif isinstance(member.type, AbstractSequence)]@
  return &(ros_message->@(member.name).data[index]);
@[        end if]@
}

@[        if isinstance(member.type, AbstractSequence)]@
int32_t @(msg_typename)__getsize_field_@(member.name)_message(void *message_handle) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return ros_message->@(member.name).size;
}

bool @(msg_typename)__init_sequence_field_@(member.name)_message(void *message_handle, int32_t size) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (ros_message->@(member.name).data) {
    allocator.deallocate(ros_message->@(member.name).data, allocator.state);
    ros_message->@(member.name).data = NULL;
    ros_message->@(member.name).size = 0;
    ros_message->@(member.name).capacity = 0;
  }

  @(idl_type_to_c(member.type.value_type)) * data = NULL;
  if (size) {
    data = (@(idl_type_to_c(member.type.value_type)) *)allocator.zero_allocate(
      size, sizeof(@(idl_type_to_c(member.type.value_type))), allocator.state);
    if (!data) {
      return false;
    }
  }
  
  ros_message->@(member.name).data = data;
  ros_message->@(member.name).size = size;
  ros_message->@(member.name).capacity = size;
  return true;
}
@[        end if]@

@[        if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == BOOLEAN_TYPE]@
@# Special handling for marshaling bool as int32_t
void @(msg_typename)__write_field_@(member.name)(void *message_handle, int32_t /* bool */ value)
{
  bool * ros_message = (bool *)message_handle;
  *ros_message = value != 0;
}

int32_t /* bool */ @(msg_typename)__read_field_@(member.name)(void *message_handle)
{
  bool * ros_message = (bool *)message_handle;
  return (*ros_message) ? 1 : 0;
}
@[        elif isinstance(member.type.value_type, BasicType)]@
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
  rosidl_runtime_c__String * ros_message = (rosidl_runtime_c__String *)message_handle;
  rosidl_runtime_c__String__assign(ros_message, value); 
}

@(msg_type_to_c(member.type.value_type)) @(msg_typename)__read_field_@(member.name)(void *message_handle)
{
  rosidl_runtime_c__String * ros_message = (rosidl_runtime_c__String *)message_handle;
  return ros_message->data;
}
@[        end if]@

@[    elif isinstance(member.type, AbstractWString)]@
// TODO: Unicode types are not supported
@[    elif isinstance(member.type, BasicType) and member.type.typename == BOOLEAN_TYPE]@
@# Special handling for marshaling bool as int32_t
int32_t /* bool */ @(msg_typename)__read_field_@(member.name)(void * message_handle) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  return (ros_message->@(member.name)) ? 1 : 0;
}
void @(msg_typename)__write_field_@(member.name)(void * message_handle, int32_t /* bool */ value) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  ros_message->@(member.name) = value != 0;
}
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
