#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

#include <@(spec.base_type.pkg_name)/@(subfolder)/@(module_name).h>
#include "rosidl_generator_c/message_type_support_struct.h"

#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

#include <rosidl_generator_c/primitives_array.h>
#include <rosidl_generator_c/primitives_array_functions.h>

@{
struct_type_name = 'rcldotnet_{}_{}_{}_t'.format(spec.base_type.pkg_name, subfolder, type_name)
msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
header_filename = "{0}/{1}/rcldotnet_{2}.h".format(package_name, subfolder, convert_camel_case_to_lower_case_underscore(type_name))
}@

#include "@(header_filename)"

void * native_create_native_message() {
   @(msg_typename) * ros_message = @(msg_typename)__create();
   return ros_message;
}

const void * native_get_typesupport() {
  const void * ptr = ROSIDL_GET_MSG_TYPE_SUPPORT(@(spec.base_type.pkg_name), @(subfolder), @(spec.msg_name));
  return ptr;
}

@[for field in spec.fields]@
@[    if field.type.is_array]@
void * native_get_field_@(field.name)_message(void *message_handle, int index) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[        if field.type.is_dynamic_array()]@
  return &(ros_message->@(field.name).data[index]);
@[        else]@
  return &(ros_message->@(field.name)[index]);
@[        end if]@
}

void * native_init_field_@(field.name)_message(void *message_handle, int size) {
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
}@
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[        if field.type.is_primitive_type() or field.type.type == 'string']@
  @(msg_typename)__Array__init(&(ros_message->@(field.name)), size);
@[        else]@
  @(nested_type)__Array__init(&(ros_message->@(field.name)), size);
@[        end if]@
}
@[        if field.type.is_primitive_type()]@
void native_write_field_@(field.name)(void *message_handle, @(primitive_msg_type_to_c(field.type.type)) value)
{
  @(primitive_msg_type_to_c(field.type.type))  * ros_message_ptr = (@(msg_typename) *)message_handle;
@[            if field.type.type == 'string']@
  rosidl_generator_c__String__assign(
    ros_message_ptr, value);
@[            else]@
  *ros_message_ptr = value;
@[            end if]@
}
@[        end if]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@(primitive_msg_type_to_c(field.type.type)) native_read_field_@(field.name)(void * message_handle) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[            if field.type.type == 'string']@
  return ros_message->@(field.name).data;
@[            else]@
  return ros_message->@(field.name);
@[            end if]@

}

void native_write_field_@(field.name)(void * message_handle, @(primitive_msg_type_to_c(field.type.type)) value) {
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[            if field.type.type == 'string']@
    rosidl_generator_c__String__assign(
      &ros_message->@(field.name), value);
@[            else]@
  ros_message->@(field.name) = value;
@[            end if]@
}
@[        else]@

void * native_get_field_@(field.name)_message(void * raw_ros_message) {
  @(msg_typename) * ros_message = (@(msg_typename) *)raw_ros_message;
  return &(ros_message->@(field.name));
}
@[        end if]@
@[    end if]@
@[end for]@

void native_destroy_native_message(void * raw_ros_message) {
  @(msg_typename) * ros_message = raw_ros_message;
  @(msg_typename)__destroy(ros_message);
}
