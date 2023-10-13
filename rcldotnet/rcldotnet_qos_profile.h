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

#ifndef RCLDOTNET_QOS_PROFILE_H
#define RCLDOTNET_QOS_PROFILE_H

#include "rcldotnet_macros.h"

// The below profile getters are intentionally provided as const pointers to avoid construction of copies.

RCLDOTNET_EXPORT
const rmw_qos_profile_t * RCLDOTNET_CDECL native_rcl_qos_get_profile_default();

RCLDOTNET_EXPORT
const rmw_qos_profile_t * RCLDOTNET_CDECL native_rcl_qos_get_profile_parameter_events();

RCLDOTNET_EXPORT
const rmw_qos_profile_t * RCLDOTNET_CDECL native_rcl_qos_get_profile_parameters();

RCLDOTNET_EXPORT
const rmw_qos_profile_t * RCLDOTNET_CDECL native_rcl_qos_get_profile_sensor_data();

RCLDOTNET_EXPORT
const rmw_qos_profile_t * RCLDOTNET_CDECL native_rcl_qos_get_profile_services_default();

RCLDOTNET_EXPORT
const rmw_qos_profile_t * RCLDOTNET_CDECL native_rcl_qos_get_profile_system_default();

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_qos_profile_read_history(void *qos_profile_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_qos_profile_read_depth(void *qos_profile_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_qos_profile_read_reliability(void *qos_profile_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_qos_profile_read_durability(void *qos_profile_handle);

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_qos_profile_read_deadline(void *qos_profile_handle, void *sec, void *nsec);

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_qos_profile_read_lifespan(void *qos_profile_handle, void *sec, void *nsec);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_qos_profile_read_liveliness(void *qos_profile_handle);

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_qos_profile_read_liveliness_lease_duration(void *qos_profile_handle, void *sec, void *nsec);

RCLDOTNET_EXPORT
int32_t /* bool */ RCLDOTNET_CDECL native_rcl_qos_profile_read_avoid_ros_namespace_conventions(void *qos_profile_handle);

#endif // RCLDOTNET_QOS_PROFILE_H
