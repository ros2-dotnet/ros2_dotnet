// Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
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

#ifndef RCLDOTNET_H
#define RCLDOTNET_H

#include "rcldotnet_macros.h"

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_init(int argc, const char *argv[]);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_clock_handle(void **clock_handle, int32_t clock_type);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_clock_handle(void *clock_handle);

RCLDOTNET_EXPORT
const char * RCLDOTNET_CDECL native_rcl_get_rmw_identifier();

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_get_error_string(char *buffer, int32_t bufferSize);

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_reset_error(void);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_ok();

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_node_handle(void **, const char *, const char *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_node_handle(void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_guard_condition_handle(void **guard_condition_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_guard_condition_handle(void *guard_condition_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_wait_set_handle(
    void **wait_set_handle,
    int32_t numberOfSubscriptions,
    int32_t numberOfGuardConditions,
    int32_t numberOfTimers,
    int32_t numberOfClients,
    int32_t numberOfServices,
    int32_t numberOfEvents);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_wait_set_handle(void *wait_set_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_clear(void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_subscription(void *, void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_service(void *wait_set_handle, void *service_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_client(void *wait_set_handle, void *client_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_timer(void *wait_set_handle, void *timer_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_guard_condition(void *wait_set_handle, void *guard_condition_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_client_wait_set_get_num_entries(
    void *action_client_handle,
    int32_t *num_subscriptions,
    int32_t *num_guard_conditions,
    int32_t *num_timers,
    int32_t *num_clients,
    int32_t *num_services);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_wait_set_add_action_client(void *wait_set_handle, void *action_client_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_server_wait_set_get_num_entries(
    void *action_server_handle,
    int32_t *num_subscriptions,
    int32_t *num_guard_conditions,
    int32_t *num_timers,
    int32_t *num_clients,
    int32_t *num_services);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_wait_set_add_action_server(void *wait_set_handle, void *action_server_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait(void *, int64_t);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_subscription_ready(void *wait_set_handle, int32_t index);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_client_ready(void *wait_set_handle, int32_t index);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_timer_ready(void *wait_set_handle, int32_t index);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_service_ready(void *wait_set_handle, int32_t index);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_guard_condition_ready(void *wait_set_handle, int32_t index);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_client_wait_set_get_entities_ready(
    void *wait_set_handle,
    void *action_client_handle,
    int32_t *is_feedback_ready,
    int32_t *is_status_ready,
    int32_t *is_goal_response_ready,
    int32_t *is_cancel_response_ready,
    int32_t *is_result_response_ready);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_server_wait_set_get_entities_ready(
    void *wait_set_handle,
    void *action_server_handle,
    int32_t *is_goal_request_ready,
    int32_t *is_cancel_request_ready,
    int32_t *is_result_request_ready,
    int32_t *is_goal_expired);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_take(void *, void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_request_id_handle(void **request_id_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_request_id_handle(void *request_id_handle);

RCLDOTNET_EXPORT
int64_t RCLDOTNET_CDECL native_rcl_request_id_get_sequence_number(void *request_id_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_take_request(void *service_handle, void *request_header_handle, void *request_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_send_response(void *service_handle, void *request_header_handle, void *resopnse_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_take_response(void *client_handle, void *request_header_handle, void *response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_feedback(void *action_client_handle, void *feedback_message_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_status(void *action_client_handle, void *status_message_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_goal_response(void *action_client_handle, void *request_header_handle, void *goal_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_cancel_response(void *action_client_handle, void *request_header_handle, void *cancel_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_result_response(void *action_client_handle, void *request_header_handle, void *result_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_goal_request(void *action_server_handle, void *request_header_handle, void *goal_request_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_send_goal_response(void *action_server_handle, void *request_header_handle, void *goal_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_accept_new_goal(void **action_goal_handle_handle, void *action_server_handle, void *goal_info_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_destroy_goal_handle(void *action_goal_handle_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_update_goal_state(void *action_goal_handle_handle, int32_t goal_event);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_publish_status(void *action_server_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_publish_feedback(void *action_server_handle, void *feedback_message_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_cancel_request(void *action_server_handle, void *request_header_handle, void *cancel_request_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_process_cancel_request(void *action_server_handle, void *cancel_request_handle, void *cancel_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_send_cancel_response(void *action_server_handle, void *request_header_handle, void *cancel_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_take_result_request(void *action_server_handle, void *request_header_handle, void *result_request_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_send_result_response(void *action_server_handle, void *request_header_handle, void *result_response_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_notify_goal_done(void *action_server_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_expire_goals(void *action_server_handle, void *goal_info_handle, int32_t *num_expired);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_goal_handle_is_active(void *action_goal_handle_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_goal_handle_get_status(void *action_goal_handle_handle, int8_t *status);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_qos_profile_handle(void **qos_profile_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_qos_profile_handle(void *qos_profile_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_write_to_qos_profile_handle(
    void *qos_profile_handle,
    int32_t history,
    int32_t depth,
    int32_t reliability,
    int32_t durability,
    uint64_t deadline_sec,
    uint64_t deadline_nsec,
    uint64_t lifespan_sec,
    uint64_t lifespan_nsec,
    int32_t liveliness,
    uint64_t liveliness_lease_duration_sec,
    uint64_t liveliness_lease_duration_nsec,
    int32_t /* bool */ avoid_ros_namespace_conventions);

RCLDOTNET_EXPORT
const rmw_qos_profile_t RCLDOTNET_CDECL native_rcl_qos_get_profile_default();

RCLDOTNET_EXPORT
const rmw_qos_profile_t RCLDOTNET_CDECL native_rcl_qos_get_profile_parameter_events();

RCLDOTNET_EXPORT
const rmw_qos_profile_t RCLDOTNET_CDECL native_rcl_qos_get_profile_parameters();

RCLDOTNET_EXPORT
const rmw_qos_profile_t RCLDOTNET_CDECL native_rcl_qos_get_profile_sensor_data();

RCLDOTNET_EXPORT
const rmw_qos_profile_t RCLDOTNET_CDECL native_rcl_qos_get_profile_services_default();

RCLDOTNET_EXPORT
const rmw_qos_profile_t RCLDOTNET_CDECL native_rcl_qos_get_profile_system_default();

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_timer_handle(void **timer_handle, void *clock_handle, int64_t period, rcl_timer_callback_t callback);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_timer_handle(void *timer_handle);

#endif // RCLDOTNET_H
