// generated from rosidl_generator_cs/resource/_msg_support.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating _<msg>_s.c files
@#
@# Context:
@#  - module_name
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - convert_camel_case_to_lower_case_underscore (function)
@#  - primitive_msg_type_to_c (function)
@#######################################################################
@
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <rosidl_generator_c/visibility_control.h>

#include <@(spec.base_type.pkg_name)/@(subfolder)/@(module_name)__struct.h>
#include <@(spec.base_type.pkg_name)/@(subfolder)/@(module_name)__functions.h>

@{
have_not_included_primitive_arrays = True
have_not_included_string = True
nested_array_dict = {}
}@
@[for field in spec.fields]@
@[  if field.type.is_array and have_not_included_primitive_arrays]@
@{have_not_included_primitive_arrays = False}@
#include <rosidl_generator_c/primitives_sequence.h>
#include <rosidl_generator_c/primitives_sequence_functions.h>
@[  end if]@
@[  if field.type.type == 'string' and have_not_included_string]@
@{have_not_included_string = False}@
#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

@[  end if]@
@{
if not field.type.is_primitive_type() and field.type.is_array:
    if field.type.type not in nested_array_dict:
        nested_array_dict[field.type.type] = field.type.pkg_name
}@
@[if nested_array_dict != {}]@
// Nested array functions includes
@[  for key in nested_array_dict]@
#include <@(nested_array_dict[key])/msg/@convert_camel_case_to_lower_case_underscore(key)__functions.h>
@[  end for]@
// end nested array functions include
@[end if]@
@[end for]@

@{
msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
}@
@[for field in spec.fields]@
@[  if field.type.is_array and (field.type.array_size is None or field.type.is_upper_bound)]@
// Get array size @(field.name)
ROSIDL_GENERATOR_C_EXPORT
int @(module_name)_native_get_size_@(field.name)(void * message_handle)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
  size_t size = ros_message->@(field.name).size;
  return (int)size;
}
@[  end if]
@[if field.type.is_array and field.type.is_primitive_type()]@
// Get array data @(field.name)
ROSIDL_GENERATOR_C_EXPORT
@[  if field.type.type == 'string']@
const char * @(module_name)_native_get_string_by_index_@(field.name)(void * message_handle, int index)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[    if field.type.array_size is None or field.type.is_upper_bound]@
  @(primitive_msg_type_to_c(field.type.type)) * data_ptr = ros_message->@(field.name).data;
@[    else]
  @(primitive_msg_type_to_c(field.type.type)) * data_ptr = ros_message->@(field.name);
@[    end if]
  return (const char *)data_ptr[index].data;
}
@[  else]
@(primitive_msg_type_to_c(field.type.type)) * @(module_name)_native_get_array_ptr_@(field.name)(void * message_handle)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[    if field.type.array_size is None or field.type.is_upper_bound]@
  @(primitive_msg_type_to_c(field.type.type)) * data_ptr = ros_message->@(field.name).data;
@[    else]
  @(primitive_msg_type_to_c(field.type.type)) * data_ptr = ros_message->@(field.name);
@[    end if]
  return data_ptr;
}
@[  end if]
@[elif field.type.is_array]@
// Get data from array of nested types @(field.name)
ROSIDL_GENERATOR_C_EXPORT
void * @(module_name)_native_get_handle_by_index_@(field.name)(void * message_handle, int index)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;

@[    if field.type.array_size is None or field.type.is_upper_bound]@
    return &(ros_message->@(field.name).data[index]);
@[    else]
    //TODO(sam): add support for static arrays
    return 0;
@[    end if]
}
@[elif field.type.is_primitive_type()]@
// Get primitive type @(field.name)
ROSIDL_GENERATOR_C_EXPORT
@[  if field.type.type == 'string']@
const char * @(module_name)_native_read_field_@(field.name)(void * message_handle)
@[  else]
@(primitive_msg_type_to_c(field.type.type)) @(module_name)_native_read_field_@(field.name)(void * message_handle)
@[  end if]
{
@[  if field.type.is_primitive_type()]@
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[    if field.type.type == 'string']@
  return ros_message->@(field.name).data;
@[    else]@
  return ros_message->@(field.name);
@[    end if]@
@[  end if]@
}
@[end if]@
@[end for]@
@[for field in spec.fields]@
@[if field.type.is_array]@
// Set array @(field.name)

@[  if field.type.type == 'string']@
@# Set string array
ROSIDL_GENERATOR_C_EXPORT
bool @(module_name)_native_set_array_@(field.name)(void * message_handle, const char * data[], int size)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[    if field.type.array_size is None or field.type.is_upper_bound]@
  if(!rosidl_generator_c__String__Sequence__init(&(ros_message->@(field.name)), size))
  {
    return false;
  }
  @(primitive_msg_type_to_c(field.type.type)) * dest = ros_message->@(field.name).data;
@[    else]
  @(primitive_msg_type_to_c(field.type.type)) * dest = ros_message->@(field.name);
@[    end if]

  for(int i = 0; i < size; i++)
  {
    rosidl_generator_c__String__assign(&dest[i], data[i]);
  }
  return true;
}
@[  elif field.type.is_primitive_type()]@
@# Set primitive array
ROSIDL_GENERATOR_C_EXPORT
bool @(module_name)_native_set_array_@(field.name)(void * message_handle, const @(primitive_msg_type_to_c(field.type.type)) * data, int size)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[    if field.type.array_size is None or field.type.is_upper_bound]@
  if(!rosidl_generator_c__@(field.type.type)__Sequence__init(&(ros_message->@(field.name)), size))
  {
    return false;
  }
  @(primitive_msg_type_to_c(field.type.type)) * dest = ros_message->@(field.name).data;
@[    else]
  @(primitive_msg_type_to_c(field.type.type)) * dest = ros_message->@(field.name);
@[    end if]
  memcpy(dest, data, sizeof(@(primitive_msg_type_to_c(field.type.type)))*size);
  return true;
}
@[  else]@
@# Set array of nested types
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
lowercase_field_type = convert_camel_case_to_lower_case_underscore(field.type.type)
}@

ROSIDL_GENERATOR_C_EXPORT
bool @(module_name)_native_set_array_@(field.name)(void * message_handle, const void * handle_array[], int size)
{
  //TODO(sam): Add suport for copying existing array of nested messages to this message
  return false;
}

@# Init array of nested types
ROSIDL_GENERATOR_C_EXPORT
bool @(module_name)_native_init_nested_array_@(field.name)(void * message_handle, int size)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;

  if(!@(nested_type)__Sequence__init(&(ros_message->@(field.name)), size))
  {
    return false;
  }

  return true;
}

@[  end if]@

@[elif field.type.is_primitive_type()]@
// Set primitive type @(field.name)
ROSIDL_GENERATOR_C_EXPORT
@[  if field.type.type == 'string']@
void @(module_name)_native_write_field_@(field.name)(void * message_handle, const char * value)
@[  else]
void @(module_name)_native_write_field_@(field.name)(void * message_handle, @(primitive_msg_type_to_c(field.type.type)) value)
@[  end if]
{
@[  if field.type.is_primitive_type()]@
  @(msg_typename) * ros_message = (@(msg_typename) *)message_handle;
@[    if field.type.type == 'string']
  rosidl_generator_c__String__assign(&ros_message->@(field.name), value);
@[    else]@
  ros_message->@(field.name) = value;
@[    end if]@
@[  end if]@
}
@[end if]@
@[end for]@
@[for field in spec.fields]@
@[if not field.type.is_array and not field.type.is_primitive_type()]
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
lowercase_field_type = convert_camel_case_to_lower_case_underscore(field.type.type)
}@
ROSIDL_GENERATOR_C_EXPORT
void * @(module_name)_get_nested_message_handle_@(field.name)(void * raw_ros_message)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)raw_ros_message;
  @(nested_type) * nested_message = &(ros_message->@(field.name));
  return (void *)nested_message;
}
@[end if]@
@[end for]@
