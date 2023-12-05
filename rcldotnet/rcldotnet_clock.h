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

#ifndef RCLDOTNET_CLOCK_H
#define RCLDOTNET_CLOCK_H

#include "rcldotnet_macros.h"

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_clock_get_now(void *clock_handle, rcl_time_point_value_t *time_point);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_enable_ros_time_override(void *clock_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_disable_ros_time_override(void *clock_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_set_ros_time_override(void *clock_handle, rcl_time_point_value_t time_point_value);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_clock_add_jump_callback(void *clock_handle, rcl_jump_threshold_t threshold, rcl_jump_callback_t callback);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_clock_remove_jump_callback(void *clock_handle, rcl_jump_callback_t callback);

#endif // RCLDOTNET_CLOCK_H
