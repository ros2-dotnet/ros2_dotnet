// Copyright 2023 Queensland University of Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rcl/rcl.h>
#include <rcl_yaml_param_parser/parser.h>

#include <rosidl_runtime_c/primitives_sequence.h>

#include <rcl_interfaces/msg/parameter_value.h>
#include <rcl_interfaces/msg/parameter_type.h>

#include "rcldotnet_params.h"

typedef struct rcl_void_array_s {
  /// Array with values
  void *values;
  /// Number of values in the array
  size_t size;
} rcl_void_array_t;

ROSIDL_RUNTIME_C__PRIMITIVE_SEQUENCE(void, void)

void native_rcl_destroy_rcl_params(void *rcl_params) {
  rcl_yaml_node_struct_fini((rcl_params_t *)rcl_params);
}

void rcldotnet_params_copy_to_rosidl_runtime_c__String(rosidl_runtime_c__String *dest, const char * src) {
  size_t length = strlen(src);
  size_t capacity = sizeof(char) * (length + 1);
  dest->size = length;

  if (dest->capacity != capacity) {
    if (dest->capacity != 0) {
      free(dest->data);
    }

    dest->capacity = capacity;
    dest->data = malloc(capacity);
  }

  memcpy(dest->data, src, capacity);
}

void rcldotnet_params_copy_yaml_array_to_parameter_array(rosidl_runtime_c__void__Sequence *dest, const rcl_void_array_t *src, size_t element_size) {
  size_t length = src->size;
  size_t length_bytes = length * element_size;
  dest->size = length;

  if (dest->capacity != length) {
    if (dest->capacity > 0) {
      free(dest->data);
    }
    
    dest->capacity = length;
    dest->data = malloc(length_bytes);
  }

  memcpy(dest->data, src->values, length_bytes);
}

void rcldotnet_params_copy_yaml_string_array_to_parameter_string_array(rosidl_runtime_c__String__Sequence *dest, rcutils_string_array_t *src) {
  size_t length = src->size;

  if (dest->capacity != length) {
    
    if (dest->capacity != 0) {
      for (int i = 0; i < dest->capacity; i++) {
        free(dest->data->data);
      }

      free(dest->data);
    }
    
    dest->capacity = length;
    dest->data = malloc(length * sizeof(rosidl_runtime_c__String));
    
    // If the elements aren't initialised, free may be run on an uninitialised data pointer.
    for (int i = 0; i < length; i++) {
      dest->data[i].capacity = 0;
      dest->data[i].size = 0;
    }
  }

  dest->size = length;

  for (int i = 0; i < length; i++) {
    rosidl_runtime_c__String *dest_element = &dest->data[i];
    rcldotnet_params_copy_to_rosidl_runtime_c__String(&(dest->data[i]), src->data[i]);
  }
}

bool rcldotnet_params_try_get_parameter_from_node_params(const rcl_node_params_t *node_params, const char *name, rcl_interfaces__msg__ParameterValue *param_value) {
  int param_index = 0;
  for (; param_index < node_params->num_params; param_index++) {
    if (strcmp(name, node_params->parameter_names[param_index]) == 0) {
      break;
    }
  }

  if (param_index >= node_params->num_params) {
    return false;
  }

  rcl_variant_t *rcl_param_value = &node_params->parameter_values[param_index];

  switch (param_value->type) {
    case rcl_interfaces__msg__ParameterType__PARAMETER_BOOL:
      param_value->bool_value = *rcl_param_value->bool_value;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER:
      param_value->integer_value = *rcl_param_value->integer_value;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE:
      param_value->double_value = *rcl_param_value->double_value;
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_STRING:
      rcldotnet_params_copy_to_rosidl_runtime_c__String(&param_value->string_value, rcl_param_value->string_value);
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_BYTE_ARRAY:
      // Byte array parameter loading from YAML not implemented in RCL.
      //rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->byte_array_value, (rcl_void_array_t *)rcl_param_value->byte_array_value, sizeof(char));
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_BOOL_ARRAY:
      rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->bool_array_value, (rcl_void_array_t *)rcl_param_value->bool_array_value, sizeof(bool));
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER_ARRAY:
      rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->integer_array_value, (rcl_void_array_t *)rcl_param_value->integer_array_value, sizeof(int64_t));
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE_ARRAY:
      rcldotnet_params_copy_yaml_array_to_parameter_array((rosidl_runtime_c__void__Sequence *)&param_value->double_array_value, (rcl_void_array_t *)rcl_param_value->double_array_value, sizeof(double));
    break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_STRING_ARRAY:
      rcldotnet_params_copy_yaml_string_array_to_parameter_string_array(&param_value->string_array_value, rcl_param_value->string_array_value);
    break;
  }

  return true;
}

int32_t /* bool */ native_rcl_try_get_parameter(void *param_value_handle, const void *params_handle, const void *node_handle, const char *name) {
  if (params_handle == NULL) return false;

  rcl_interfaces__msg__ParameterValue *param_value = (rcl_interfaces__msg__ParameterValue *)param_value_handle;
  const rcl_params_t *rcl_params = (const rcl_params_t *)params_handle;
  const rcl_node_t *node = (const rcl_node_t *)node_handle;
  const char *node_name = rcl_node_get_fully_qualified_name(node);

  // First check if there is an override which matches the fully qualified node name.
  for (int i = 0; i < rcl_params->num_nodes; i++) {
    if (strcmp(node_name, rcl_params->node_names[i]) == 0) {
      if (rcldotnet_params_try_get_parameter_from_node_params(&rcl_params->params[i], name, param_value)) {
        return true;
      }
    }
  }
  
  // Then check if there is a global override.
  for (int i = 0; i < rcl_params->num_nodes; i++) {
    if (strcmp("/**", rcl_params->node_names[i]) == 0) {
      if (rcldotnet_params_try_get_parameter_from_node_params(&rcl_params->params[i], name, param_value)) {
        return true;
      }
    }
  }

  return false;
}