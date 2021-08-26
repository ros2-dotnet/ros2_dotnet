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
#include <rmw/rmw.h>

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rcldotnet.h"

static rcl_context_t context;

int32_t native_rcl_init() {
  // TODO(esteve): parse args
  int num_args = 0;
  context = rcl_get_zero_initialized_context();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (RCL_RET_OK != ret) {
    return ret;
  }
  const char ** arg_values = NULL;
  ret = rcl_init(num_args, arg_values, &init_options, &context);
  return ret;
}

const char *native_rcl_get_rmw_identifier() {
  return rmw_get_implementation_identifier();
}

const char *native_rcl_get_error_string() {
  return rcl_get_error_string().str;
}

void native_rcl_reset_error() { rcl_reset_error(); }

bool native_rcl_ok() { return rcl_context_is_valid(&context); }

int32_t native_rcl_create_node_handle(void **node_handle, const char *name, const char *namespace) {
  rcl_node_t *node = (rcl_node_t *)malloc(sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();

  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, name, namespace, &context, &default_options);
  *node_handle = (void *)node;
  return ret;
}

void *native_rcl_get_zero_initialized_wait_set() {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)malloc(sizeof(rcl_wait_set_t));
  *wait_set = rcl_get_zero_initialized_wait_set();
  return (void *)wait_set;
}

int32_t native_rcl_wait_set_init(
    void *wait_set_handle,
    long number_of_subscriptions,
    long number_of_guard_conditions,
    long number_of_timers,
    long number_of_clients,
    long number_of_services,
    long number_of_events) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  rcl_ret_t ret = rcl_wait_set_init(
      wait_set, number_of_subscriptions, number_of_guard_conditions,
      number_of_timers, number_of_clients, number_of_services,
      number_of_events, &context, rcl_get_default_allocator());

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

void native_rcl_destroy_wait_set(void *wait_set_handle) {
  free((rcl_wait_set_t *)wait_set_handle);
}

int32_t native_rcl_wait_set(void *wait_set_handle, long timeout) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait(wait_set, timeout);

  return ret;
}

int32_t native_rcl_take(void *subscription_handle, void *message_handle) {
  rcl_subscription_t * subscription = (rcl_subscription_t *)subscription_handle;

  rcl_ret_t ret = rcl_take(subscription, message_handle, NULL, NULL);
  return ret;
}

void * native_rcl_create_request_header_handle(void) {
  rmw_request_id_t *request_header = (rmw_request_id_t *)malloc(sizeof(rmw_request_id_t));
  memset(request_header, 0, sizeof(rmw_request_id_t));
  return (void *)request_header;
}

void native_rcl_destroy_request_header_handle(void *request_header_handle) {
  free((rmw_request_id_t *)request_header_handle);
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

int32_t native_rcl_wait(void *wait_set_handle, int64_t timeout) {
  rcl_wait_set_t * wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait(wait_set, timeout);

  return ret;
}
