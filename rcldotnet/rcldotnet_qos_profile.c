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

#include "rcldotnet_qos_profile.h"

const rmw_qos_profile_t * native_rcl_qos_get_const_profile_default() {
  return &rmw_qos_profile_default;
}

const rmw_qos_profile_t * native_rcl_qos_get_const_profile_parameter_events() {
  return &rmw_qos_profile_parameter_events;
}

const rmw_qos_profile_t * native_rcl_qos_get_const_profile_parameters() {
  return &rmw_qos_profile_parameters;
}

const rmw_qos_profile_t * native_rcl_qos_get_const_profile_sensor_data() {
  return &rmw_qos_profile_sensor_data;
}

const rmw_qos_profile_t * native_rcl_qos_get_const_profile_services_default() {
  return &rmw_qos_profile_services_default;
}

const rmw_qos_profile_t * native_rcl_qos_get_const_profile_system_default() {
  return &rmw_qos_profile_system_default;
}

int32_t native_rcl_qos_profile_read_history(void *qos_profile_handle) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  return (int32_t)qos_profile->history;
}

int32_t native_rcl_qos_profile_read_depth(void *qos_profile_handle) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  return (int32_t)qos_profile->depth;
}

int32_t native_rcl_qos_profile_read_reliability(void *qos_profile_handle) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  return (int32_t)qos_profile->reliability;
}

int32_t native_rcl_qos_profile_read_durability(void *qos_profile_handle) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  return (int32_t)qos_profile->durability;
}

void rcldotnet_qos_profile_read_rmw_time(rmw_time_t *time, void *sec, void *nsec) {
  int64_t *sec_ptr = (int64_t *)sec;
  *sec_ptr = time->sec;

  int64_t *nsec_ptr = (int64_t *)nsec;
  *nsec_ptr = time->nsec;
}

void native_rcl_qos_profile_read_deadline(void *qos_profile_handle, void *sec, void *nsec) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  rcldotnet_qos_profile_read_rmw_time(&qos_profile->deadline, sec, nsec);
}

void native_rcl_qos_profile_read_lifespan(void *qos_profile_handle, void *sec, void *nsec) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  rcldotnet_qos_profile_read_rmw_time(&qos_profile->lifespan, sec, nsec);
}

int32_t native_rcl_qos_profile_read_liveliness(void *qos_profile_handle) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  return (int32_t)qos_profile->liveliness;
}

void native_rcl_qos_profile_read_liveliness_lease_duration(void *qos_profile_handle, void *sec, void *nsec) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  rcldotnet_qos_profile_read_rmw_time(&qos_profile->liveliness_lease_duration, sec, nsec);
}

int32_t /* bool */ native_rcl_qos_profile_read_avoid_ros_namespace_conventions(void *qos_profile_handle) {
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  return qos_profile->avoid_ros_namespace_conventions;
}
