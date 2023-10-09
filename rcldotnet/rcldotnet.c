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

#include <assert.h>
#include <stdlib.h>

#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rmw/rmw.h>

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rcldotnet.h"

static rcl_context_t context;

int32_t native_rcl_init(int argc, const char *argv[]) {
  context = rcl_get_zero_initialized_context();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (RCL_RET_OK != ret) {
    return ret;
  }
  ret = rcl_init(argc, argv, &init_options, &context);
  if (ret != RCL_RET_OK) {
    return ret;
  }

  return ret;
}

int32_t native_rcl_create_clock_handle(void **clock_handle, int32_t clock_type) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_clock_t *clock = malloc(sizeof(rcl_clock_t));

  rcl_ret_t ret = rcl_clock_init((rcl_clock_type_t)clock_type, clock, &allocator);
  
  *clock_handle = (void *)clock;
  return ret;
}

int32_t native_rcl_destroy_clock_handle(void *clock_handle) {
  rcl_clock_t *clock = (rcl_clock_t *)clock_handle;

  rcl_ret_t ret = rcl_clock_fini(clock);
  free(clock);

  return ret;
}

const char *native_rcl_get_rmw_identifier() {
  return rmw_get_implementation_identifier();
}

void native_rcl_get_error_string(char *buffer, int32_t bufferSize) {
  size_t minBufferSize = (size_t)bufferSize < (size_t)RCUTILS_ERROR_MESSAGE_MAX_LENGTH
    ? (size_t)bufferSize
    : (size_t)RCUTILS_ERROR_MESSAGE_MAX_LENGTH;

  strncpy(buffer, rcl_get_error_string().str, minBufferSize);
}

void native_rcl_reset_error(void) {
  rcl_reset_error();
}

int32_t native_rcl_ok() {
  bool result = rcl_context_is_valid(&context);
  return result ? 1 : 0;
}

int32_t native_rcl_create_node_handle(void **node_handle, const char *name, const char *namespace) {
  rcl_node_t *node = (rcl_node_t *)malloc(sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();

  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, name, namespace, &context, &default_options);
  *node_handle = (void *)node;
  return ret;
}

int32_t native_rcl_destroy_node_handle(void *node_handle) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rcl_ret_t ret = rcl_node_fini(node);
  free(node);

  return ret;
}

int32_t native_rcl_create_guard_condition_handle(void **guard_condition_handle) {
  rcl_guard_condition_t *guard_condition =
      (rcl_guard_condition_t *)malloc(sizeof(rcl_guard_condition_t));
  *guard_condition = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t guard_condition_ops =
      rcl_guard_condition_get_default_options();

  rcl_ret_t ret =
      rcl_guard_condition_init(guard_condition, &context, guard_condition_ops);

  *guard_condition_handle = (void *)guard_condition;

  return ret;
}

int32_t native_rcl_destroy_guard_condition_handle(void *guard_condition_handle) {
  rcl_guard_condition_t *guard_condition = (rcl_guard_condition_t *)guard_condition_handle;

  rcl_ret_t ret = rcl_guard_condition_fini(guard_condition);
  free(guard_condition);

  return ret;
}

int32_t native_rcl_create_wait_set_handle(
    void **wait_set_handle,
    int32_t number_of_subscriptions,
    int32_t number_of_guard_conditions,
    int32_t number_of_timers,
    int32_t number_of_clients,
    int32_t number_of_services,
    int32_t number_of_events) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)malloc(sizeof(rcl_wait_set_t));
  *wait_set = rcl_get_zero_initialized_wait_set();

  rcl_ret_t ret = rcl_wait_set_init(
      wait_set, number_of_subscriptions, number_of_guard_conditions,
      number_of_timers, number_of_clients, number_of_services,
      number_of_events, &context, rcl_get_default_allocator());

  *wait_set_handle = (void *)wait_set;

  return ret;
}

int32_t native_rcl_destroy_wait_set_handle(void *wait_set_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  rcl_ret_t ret = rcl_wait_set_fini(wait_set);
  free(wait_set);

  return ret;
}

int32_t native_rcl_wait_set_clear(void *wait_set_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait_set_clear(wait_set);

  return ret;
}

int32_t native_rcl_wait_set_add_subscription(void *wait_set_handle, void *subscription_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_subscription_t *subscription = (rcl_subscription_t *)subscription_handle;
  rcl_ret_t ret = rcl_wait_set_add_subscription(wait_set, subscription, NULL);

  return ret;
}

int32_t native_rcl_wait_set_add_service(void *wait_set_handle, void *service_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_service_t *service = (rcl_service_t *)service_handle;
  rcl_ret_t ret = rcl_wait_set_add_service(wait_set, service, NULL);

  return ret;
}

int32_t native_rcl_wait_set_add_client(void *wait_set_handle, void *client_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_client_t *service = (rcl_client_t *)client_handle;
  rcl_ret_t ret = rcl_wait_set_add_client(wait_set, service, NULL);

  return ret;
}

int32_t native_rcl_wait_set_add_timer(void *wait_set_handle, void *timer_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;
  rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, timer, NULL);

  return ret;
}

int32_t native_rcl_wait_set_add_guard_condition(void *wait_set_handle, void *guard_condition_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_guard_condition_t *guard_condition = (rcl_guard_condition_t *)guard_condition_handle;
  rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, guard_condition, NULL);

  return ret;
}

int32_t native_rcl_action_client_wait_set_get_num_entries(
    void *action_client_handle,
    int32_t *num_subscriptions,
    int32_t *num_guard_conditions,
    int32_t *num_timers,
    int32_t *num_clients,
    int32_t *num_services)
{
    rcl_action_client_t *action_client = (rcl_action_client_t *)action_client_handle;

    size_t num_subscriptions_as_size_t;
    size_t num_guard_conditions_as_size_t;
    size_t num_timers_as_size_t;
    size_t num_clients_as_size_t;
    size_t num_services_as_size_t;

    rcl_ret_t ret = rcl_action_client_wait_set_get_num_entities(
        action_client,
        &num_subscriptions_as_size_t,
        &num_guard_conditions_as_size_t,
        &num_timers_as_size_t,
        &num_clients_as_size_t,
        &num_services_as_size_t);

    *num_subscriptions = (int32_t)num_subscriptions_as_size_t;
    *num_guard_conditions = (int32_t)num_guard_conditions_as_size_t;
    *num_timers = (int32_t)num_timers_as_size_t;
    *num_clients = (int32_t)num_clients_as_size_t;
    *num_services = (int32_t)num_services_as_size_t;

    return ret;
}

int32_t native_rcl_action_wait_set_add_action_client(void *wait_set_handle, void *action_client_handle) {
    rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
    rcl_action_client_t *action_client = (rcl_action_client_t *)action_client_handle;
    rcl_ret_t ret = rcl_action_wait_set_add_action_client(wait_set, action_client, NULL, NULL);

    return ret;
}

int32_t native_rcl_action_server_wait_set_get_num_entries(
    void *action_server_handle,
    int32_t *num_subscriptions,
    int32_t *num_guard_conditions,
    int32_t *num_timers,
    int32_t *num_clients,
    int32_t *num_services)
{
    rcl_action_server_t *action_server = (rcl_action_server_t *)action_server_handle;

    size_t num_subscriptions_as_size_t;
    size_t num_guard_conditions_as_size_t;
    size_t num_timers_as_size_t;
    size_t num_clients_as_size_t;
    size_t num_services_as_size_t;

    rcl_ret_t ret = rcl_action_server_wait_set_get_num_entities(
        action_server,
        &num_subscriptions_as_size_t,
        &num_guard_conditions_as_size_t,
        &num_timers_as_size_t,
        &num_clients_as_size_t,
        &num_services_as_size_t);

    *num_subscriptions = (int32_t)num_subscriptions_as_size_t;
    *num_guard_conditions = (int32_t)num_guard_conditions_as_size_t;
    *num_timers = (int32_t)num_timers_as_size_t;
    *num_clients = (int32_t)num_clients_as_size_t;
    *num_services = (int32_t)num_services_as_size_t;

    return ret;
}

int32_t native_rcl_action_wait_set_add_action_server(void *wait_set_handle, void *action_server_handle) {
    rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
    rcl_action_server_t *action_server = (rcl_action_server_t *)action_server_handle;
    rcl_ret_t ret = rcl_action_wait_set_add_action_server(wait_set, action_server, NULL);

    return ret;
}

int32_t native_rcl_wait(void *wait_set_handle, int64_t timeout) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait(wait_set, timeout);

  return ret;
}

int32_t native_rcl_wait_set_subscription_ready(void *wait_set_handle, int32_t index) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  if (index >= wait_set->size_of_subscriptions)
  {
    return false;
  }

  bool result = wait_set->subscriptions[index] != NULL;
  return result ? 1 : 0;
}

int32_t native_rcl_wait_set_client_ready(void *wait_set_handle, int32_t index) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  if (index >= wait_set->size_of_clients)
  {
    return false;
  }

  bool result = wait_set->clients[index] != NULL;
  return result ? 1 : 0;
}

int32_t native_rcl_wait_set_timer_ready(void *wait_set_handle, int32_t index) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  if (index >= wait_set->size_of_timers)
  {
    return false;
  }

  bool result = wait_set->timers[index] != NULL;
  return result ? 1 : 0;
}

int32_t native_rcl_wait_set_service_ready(void *wait_set_handle, int32_t index) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  if (index >= wait_set->size_of_services)
  {
    return false;
  }

  bool result = wait_set->services[index] != NULL;
  return result ? 1 : 0;
}

int32_t native_rcl_wait_set_guard_condition_ready(void *wait_set_handle, int32_t index) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  if (index >= wait_set->size_of_guard_conditions)
  {
    return false;
  }

  bool result = wait_set->guard_conditions[index] != NULL;
  return result ? 1 : 0;
}

int32_t native_rcl_action_client_wait_set_get_entities_ready(
    void *wait_set_handle,
    void *action_client_handle,
    int32_t *is_feedback_ready,
    int32_t *is_status_ready,
    int32_t *is_goal_response_ready,
    int32_t *is_cancel_response_ready,
    int32_t *is_result_response_ready)
{
    rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
    rcl_action_client_t *action_client = (rcl_action_client_t *)action_client_handle;

    bool is_feedback_ready_as_bool;
    bool is_status_ready_as_bool;
    bool is_goal_response_ready_as_bool;
    bool is_cancel_response_ready_as_bool;
    bool is_result_response_ready_as_bool;

    rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
        wait_set,
        action_client,
        &is_feedback_ready_as_bool,
        &is_status_ready_as_bool,
        &is_goal_response_ready_as_bool,
        &is_cancel_response_ready_as_bool,
        &is_result_response_ready_as_bool);

    *is_feedback_ready = is_feedback_ready_as_bool ? 1 : 0;
    *is_status_ready = is_status_ready_as_bool ? 1 : 0;
    *is_goal_response_ready = is_goal_response_ready_as_bool ? 1 : 0;
    *is_cancel_response_ready = is_cancel_response_ready_as_bool ? 1 : 0;
    *is_result_response_ready = is_result_response_ready_as_bool ? 1 : 0;

    return ret;
}

int32_t native_rcl_action_server_wait_set_get_entities_ready(
    void *wait_set_handle,
    void *action_server_handle,
    int32_t *is_goal_request_ready,
    int32_t *is_cancel_request_ready,
    int32_t *is_result_request_ready,
    int32_t *is_goal_expired)
{
    rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
    rcl_action_server_t *action_server = (rcl_action_server_t *)action_server_handle;

    bool is_goal_request_ready_as_bool;
    bool is_cancel_request_ready_as_bool;
    bool is_result_request_ready_as_bool;
    bool is_goal_expired_as_bool;

    rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
        wait_set,
        action_server,
        &is_goal_request_ready_as_bool,
        &is_cancel_request_ready_as_bool,
        &is_result_request_ready_as_bool,
        &is_goal_expired_as_bool);

    *is_goal_request_ready = is_goal_request_ready_as_bool ? 1 : 0;
    *is_cancel_request_ready = is_cancel_request_ready_as_bool ? 1 : 0;
    *is_result_request_ready = is_result_request_ready_as_bool ? 1 : 0;
    *is_goal_expired = is_goal_expired_as_bool ? 1 : 0;

    return ret;
}

int32_t native_rcl_take(void *subscription_handle, void *message_handle) {
  rcl_subscription_t * subscription = (rcl_subscription_t *)subscription_handle;

  rcl_ret_t ret = rcl_take(subscription, message_handle, NULL, NULL);
  return ret;
}

int32_t native_rcl_create_request_id_handle(void **request_id_handle) {
  rmw_request_id_t *request_id = (rmw_request_id_t *)malloc(sizeof(rmw_request_id_t));
  memset(request_id, 0, sizeof(rmw_request_id_t));

  *request_id_handle = (void *)request_id;
  return RCL_RET_OK;
}

int32_t native_rcl_destroy_request_id_handle(void *request_id_handle) {
  free((rmw_request_id_t *)request_id_handle);
  return RCL_RET_OK;
}

int64_t native_rcl_request_id_get_sequence_number(void *request_id_handle) {
  rmw_request_id_t *request_id = (rmw_request_id_t *)request_id_handle;
  return request_id->sequence_number;
}

int32_t native_rcl_take_request(void *service_handle, void *request_header_handle, void *request_handle) {
  rcl_service_t * service = (rcl_service_t *)service_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_take_request(service, request_header, request_handle);
  return ret;
}

int32_t native_rcl_send_response(void *service_handle, void *request_header_handle, void *resopnse_handle) {
  rcl_service_t * service = (rcl_service_t *)service_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_send_response(service, request_header, resopnse_handle);
  return ret;
}

int32_t native_rcl_take_response(void *client_handle, void *request_header_handle, void *response_handle) {
  rcl_client_t * client = (rcl_client_t *)client_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_take_response(client, request_header, response_handle);
  return ret;
}

int32_t native_rcl_action_take_feedback(void *action_client_handle, void *feedback_message_handle) {
  rcl_action_client_t * action_client = (rcl_action_client_t *)action_client_handle;

  rcl_ret_t ret = rcl_action_take_feedback(action_client, feedback_message_handle);
  return ret;
}

int32_t native_rcl_action_take_status(void *action_client_handle, void *status_message_handle) {
  rcl_action_client_t * action_client = (rcl_action_client_t *)action_client_handle;

  rcl_ret_t ret = rcl_action_take_status(action_client, status_message_handle);
  return ret;
}

int32_t native_rcl_action_take_goal_response(void *action_client_handle, void *request_header_handle, void *goal_response_handle) {
  rcl_action_client_t * action_client = (rcl_action_client_t *)action_client_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_take_goal_response(action_client, request_header, goal_response_handle);
  return ret;
}

int32_t native_rcl_action_take_cancel_response(void *action_client_handle, void *request_header_handle, void *cancel_response_handle) {
  rcl_action_client_t * action_client = (rcl_action_client_t *)action_client_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_take_cancel_response(action_client, request_header, cancel_response_handle);
  return ret;
}

int32_t native_rcl_action_take_result_response(void *action_client_handle, void *request_header_handle, void *result_response_handle) {
  rcl_action_client_t * action_client = (rcl_action_client_t *)action_client_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_take_result_response(action_client, request_header, result_response_handle);
  return ret;
}

int32_t native_rcl_action_take_goal_request(void *action_server_handle, void *request_header_handle, void *goal_request_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_take_goal_request(action_server, request_header, goal_request_handle);
  return ret;
}

int32_t native_rcl_action_send_goal_response(void *action_server_handle, void *request_header_handle, void *goal_response_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_send_goal_response(action_server, request_header, goal_response_handle);
  return ret;
}

int32_t native_rcl_action_accept_new_goal(void **action_goal_handle_handle, void *action_server_handle, void *goal_info_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rcl_action_goal_info_t * goal_info = (rcl_action_goal_info_t *)goal_info_handle;

  rcl_action_goal_handle_t *action_goal_handle =
    (rcl_action_goal_handle_t *)malloc(sizeof(rcl_action_goal_handle_t));

  *action_goal_handle = rcl_action_get_zero_initialized_goal_handle();

  rcl_action_goal_handle_t *rcl_action_goal_handle = rcl_action_accept_new_goal(action_server, goal_info);
  rcl_ret_t ret;
  if (rcl_action_goal_handle == NULL)
  {
    ret = RCL_RET_ERROR;
  }
  else
  {
    // Copy out goal handle since action server storage disappears when it is fini'd.
    *action_goal_handle = *rcl_action_goal_handle;

    // Get the goal_info from the goal_handle to return the stamp back to native code.
    ret = rcl_action_goal_handle_get_info(action_goal_handle, goal_info);
  }

  *action_goal_handle_handle = (void *)action_goal_handle;

  return ret;
}

int32_t native_rcl_action_destroy_goal_handle(void *action_goal_handle) {
  rcl_action_goal_handle_t *goal_handle = (rcl_action_goal_handle_t *)action_goal_handle;

  rcl_ret_t ret = rcl_action_goal_handle_fini(goal_handle);
  free(goal_handle);

  return ret;
}

int32_t native_rcl_action_update_goal_state(void *action_goal_handle_handle, int32_t goal_event) {
  rcl_action_goal_handle_t *goal_handle = (rcl_action_goal_handle_t *)action_goal_handle_handle;

  rcl_ret_t ret = rcl_action_update_goal_state(goal_handle, (rcl_action_goal_event_t)goal_event);
  return ret;
}

int32_t native_rcl_action_publish_status(void *action_server_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;

  rcl_action_goal_status_array_t status_message =
    rcl_action_get_zero_initialized_goal_status_array();

  rcl_ret_t ret = rcl_action_get_goal_status_array(action_server, &status_message);
  if (RCL_RET_OK != ret) {
    return ret;
  }

  ret = rcl_action_publish_status(action_server, &status_message);

  rcl_ret_t cleanup_ret;
  cleanup_ret = rcl_action_goal_status_array_fini(&status_message);
  if (cleanup_ret != RCL_RET_OK)
  {
    // If we got two unexpected errors, return the earlier error.
    if (ret == RCL_RET_OK) {
      // Error message already set.
      ret = cleanup_ret;
    }
  }

  return ret;
}

int32_t native_rcl_action_publish_feedback(void *action_server_handle, void *feedback_message_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;

  rcl_ret_t ret = rcl_action_publish_feedback(action_server, feedback_message_handle);
  return ret;
}

int32_t native_rcl_action_take_cancel_request(void *action_server_handle, void *request_header_handle, void *cancel_request_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_take_cancel_request(action_server, request_header, cancel_request_handle);
  return ret;
}

int32_t native_rcl_action_process_cancel_request(void *action_server_handle, void *cancel_request_handle, void *cancel_response_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;

  // the rcl_action_cancel_request_t is a direct typedef to
  // action_msgs__srv__CancelGoal_Request

  // rcl_action_cancel_response_t is a wrapper struct around
  // action_msgs__srv__CancelGoal_Response with an additional allocator field.
  // -> Don't cast in this case!

  rcl_action_cancel_response_t tmp_cancel_response = rcl_action_get_zero_initialized_cancel_response();

  rcl_ret_t ret = rcl_action_process_cancel_request(action_server, cancel_request_handle, &tmp_cancel_response);

  // TODO: (sh) would be better to copy the list over element by element?

  // HACK: Don't deallocate but instead move reference to data into incoming cancel_response_handle.
  // rcl_action_cancel_response_fini(&cancel_response);
  action_msgs__srv__CancelGoal_Response * cancel_response = (action_msgs__srv__CancelGoal_Response *)cancel_response_handle;
  *cancel_response = tmp_cancel_response.msg;

  // HACK: as rcl_action_cancel_response_init doesn't fill in capacity...
  cancel_response->goals_canceling.capacity = cancel_response->goals_canceling.size;

  return ret;
}

int32_t native_rcl_action_send_cancel_response(void *action_server_handle, void *request_header_handle, void *cancel_response_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_send_cancel_response(action_server, request_header, cancel_response_handle);
  return ret;
}

int32_t native_rcl_action_take_result_request(void *action_server_handle, void *request_header_handle, void *result_request_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_take_result_request(action_server, request_header, result_request_handle);
  return ret;
}

int32_t native_rcl_action_send_result_response(void *action_server_handle, void *request_header_handle, void *result_response_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;
  rmw_request_id_t * request_header = (rmw_request_id_t *)request_header_handle;

  rcl_ret_t ret = rcl_action_send_result_response(action_server, request_header, result_response_handle);
  return ret;
}

int32_t native_rcl_action_notify_goal_done(void *action_server_handle) {
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;

  rcl_ret_t ret = rcl_action_notify_goal_done(action_server);
  return ret;
}

int32_t native_rcl_action_expire_goals(void *action_server_handle, void *goal_info_handle, int32_t *num_expired)
{
  rcl_action_server_t * action_server = (rcl_action_server_t *)action_server_handle;

  size_t num_expired_as_size_t;

  // Only provide one goal_info to the underlying function.
  // rclcpp does the same as only one goal is expected to expire at the same time.
  // The out parameter num_expired can be used with a loop to expire all goals.
  // This does also help to avoid implementing a new SafeHandle and accessor methods for a list of `GoalHandle`s.
  rcl_ret_t ret = rcl_action_expire_goals(action_server, goal_info_handle, 1, &num_expired_as_size_t);

  *num_expired = (int32_t)num_expired_as_size_t;

  return ret;
}

int32_t native_rcl_action_goal_handle_is_active(void *action_goal_handle_handle) {
  rcl_action_goal_handle_t *goal_handle = (rcl_action_goal_handle_t *)action_goal_handle_handle;

  bool result = rcl_action_goal_handle_is_active(goal_handle);
  return result ? 1 : 0;
}

int32_t native_rcl_action_goal_handle_get_status(void *action_goal_handle_handle, int8_t *status) {
  rcl_action_goal_handle_t *goal_handle = (rcl_action_goal_handle_t *)action_goal_handle_handle;

  rcl_action_goal_state_t status_as_rcl_type;

  rcl_ret_t ret = rcl_action_goal_handle_get_status(goal_handle, &status_as_rcl_type);

  *status = (int8_t)status_as_rcl_type;

  return ret;
}

int32_t native_rcl_create_qos_profile_handle(void **qos_profile_handle)
{
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)malloc(sizeof(rmw_qos_profile_t));
  *qos_profile = rmw_qos_profile_default;
  *qos_profile_handle = (void *)qos_profile;

  return RCL_RET_OK;
}

int32_t native_rcl_destroy_qos_profile_handle(void *qos_profile_handle)
{
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;
  free(qos_profile);

  return RCL_RET_OK;
}

int32_t native_rcl_write_to_qos_profile_handle(
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
    int32_t /* bool */ avoid_ros_namespace_conventions)
{
  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;

  // Can't name the enums for both Foxy and Humble the in code.
  // So use implicit conversions for now...
  // This breaking change was introduced in https://github.com/ros2/rmw/commit/05f973575e8e93454e39f51da6227509061ff189
  // In Foxy they are defined as `enum rmw_qos_history_policy_t { ... };
  //   -> so the type is called `enum rmw_qos_history_policy_t`
  // In Humble tey are defined as `typedef enum rmw_qos_history_policy_e { ... } rmw_qos_history_policy_t;
  //   -> so the type is called `rmw_qos_history_policy_t`

  qos_profile->history = /* (rmw_qos_history_policy_t) */ history;
  qos_profile->depth = (size_t)depth;
  qos_profile->reliability = /* (rmw_qos_reliability_policy_t) */ reliability;
  qos_profile->durability = /* (rmw_qos_durability_policy_t) */ durability;
  qos_profile->deadline.sec = deadline_sec;
  qos_profile->deadline.nsec = deadline_nsec;
  qos_profile->lifespan.sec = lifespan_sec;
  qos_profile->lifespan.nsec = lifespan_nsec;
  qos_profile->liveliness = /* (rmw_qos_liveliness_policy_t) */ liveliness;
  qos_profile->liveliness_lease_duration.sec = liveliness_lease_duration_sec;
  qos_profile->liveliness_lease_duration.nsec = liveliness_lease_duration_nsec;
  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions != 0;

  return RCL_RET_OK;
}

const rmw_qos_profile_t native_rcl_qos_get_profile_default() {
  return rmw_qos_profile_default;
}

const rmw_qos_profile_t native_rcl_qos_get_profile_parameter_events() {
  return rmw_qos_profile_parameter_events;
}

const rmw_qos_profile_t native_rcl_qos_get_profile_parameters() {
  return rmw_qos_profile_parameters;
}

const rmw_qos_profile_t native_rcl_qos_get_profile_sensor_data() {
  return rmw_qos_profile_sensor_data;
}

const rmw_qos_profile_t native_rcl_qos_get_profile_services_default() {
  return rmw_qos_profile_services_default;
}

const rmw_qos_profile_t native_rcl_qos_get_profile_system_default() {
  return rmw_qos_profile_system_default;
}

int32_t native_rcl_create_timer_handle(void **timer_handle, void *clock_handle, int64_t period, rcl_timer_callback_t callback) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_timer_t *timer = malloc(sizeof(rcl_timer_t));
  *timer = rcl_get_zero_initialized_timer();
  
  rcl_clock_t *clock = (rcl_clock_t *)clock_handle;

  rcl_ret_t ret = rcl_timer_init(timer, clock, &context, period, callback, allocator);
  
  *timer_handle = (void *)timer;
  return ret;
}

int32_t native_rcl_destroy_timer_handle(void *timer_handle) {
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;

  rcl_ret_t ret = rcl_timer_fini(timer);
  free(timer);

  return ret;
}
