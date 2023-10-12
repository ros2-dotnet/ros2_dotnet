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
#include <rmw/types.h>
#include <rcl_action/rcl_action.h>

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rcldotnet.h"
#include "rcldotnet_node.h"

int32_t native_rcl_create_publisher_handle(void **publisher_handle,
                                           void *node_handle, const char *topic,
                                           void *typesupport,
                                           void *qos_profile_handle) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;

  rosidl_message_type_support_t *ts =
      (rosidl_message_type_support_t *)typesupport;

  rcl_publisher_t *publisher =
      (rcl_publisher_t *)malloc(sizeof(rcl_publisher_t));
  *publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  if (qos_profile != NULL)
  {
    publisher_ops.qos = *qos_profile;
  }

  rcl_ret_t ret =
      rcl_publisher_init(publisher, node, ts, topic, &publisher_ops);

  *publisher_handle = (void *)publisher;

  return ret;
}

int32_t native_rcl_destroy_publisher_handle(void *publisher_handle, void *node_handle) {
  rcl_publisher_t *publisher = (rcl_publisher_t *)publisher_handle;
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rcl_ret_t ret = rcl_publisher_fini(publisher, node);
  free(publisher);

  return ret;
}

int32_t native_rcl_create_subscription_handle(void **subscription_handle,
                                              void *node_handle,
                                              const char *topic,
                                              void *typesupport,
                                              void *qos_profile_handle) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rmw_qos_profile_t *qos_profile = (rmw_qos_profile_t *)qos_profile_handle;

  rosidl_message_type_support_t *ts =
      (rosidl_message_type_support_t *)typesupport;

  rcl_subscription_t *subscription =
      (rcl_subscription_t *)malloc(sizeof(rcl_subscription_t));
  *subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_ops =
      rcl_subscription_get_default_options();

  if (qos_profile != NULL)
  {
    subscription_ops.qos = *qos_profile;
  }

  rcl_ret_t ret =
      rcl_subscription_init(subscription, node, ts, topic, &subscription_ops);

  *subscription_handle = (void *)subscription;

  return ret;
}

int32_t native_rcl_destroy_subscription_handle(void *subscription_handle, void *node_handle) {
    rcl_subscription_t *subscription = (rcl_subscription_t *)subscription_handle;
    rcl_node_t *node = (rcl_node_t *)node_handle;

    rcl_ret_t ret = rcl_subscription_fini(subscription, node);
    free(subscription);

    return ret;
}

int32_t native_rcl_create_service_handle(void **service_handle,
                                         void *node_handle,
                                         const char *service_name,
                                         void *typesupport) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rosidl_service_type_support_t *ts =
      (rosidl_service_type_support_t *)typesupport;

  rcl_service_t *service =
      (rcl_service_t *)malloc(sizeof(rcl_service_t));
  *service = rcl_get_zero_initialized_service();
  rcl_service_options_t service_ops =
      rcl_service_get_default_options();

  rcl_ret_t ret =
      rcl_service_init(service, node, ts, service_name, &service_ops);

  *service_handle = (void *)service;

  return ret;
}

int32_t native_rcl_destroy_service_handle(void *service_handle, void *node_handle) {
  rcl_service_t *service = (rcl_service_t *)service_handle;
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rcl_ret_t ret = rcl_service_fini(service, node);
  free(service);

  return ret;
}

int32_t native_rcl_create_client_handle(void **client_handle,
                                        void *node_handle,
                                        const char *service_name,
                                        void *typesupport) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rosidl_service_type_support_t *ts =
      (rosidl_service_type_support_t *)typesupport;

  rcl_client_t *client =
      (rcl_client_t *)malloc(sizeof(rcl_client_t));
  *client = rcl_get_zero_initialized_client();
  rcl_client_options_t client_ops =
      rcl_client_get_default_options();

  rcl_ret_t ret =
      rcl_client_init(client, node, ts, service_name, &client_ops);

  *client_handle = (void *)client;

  return ret;
}

int32_t native_rcl_destroy_client_handle(void *client_handle, void *node_handle) {
  rcl_client_t *client = (rcl_client_t *)client_handle;
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rcl_ret_t ret = rcl_client_fini(client, node);
  free(client);

  return ret;
}

int32_t native_rcl_action_create_client_handle(void **action_client_handle,
                                               void *node_handle,
                                               const char *action_name,
                                               void *typesupport) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rosidl_action_type_support_t *ts =
      (rosidl_action_type_support_t *)typesupport;

  rcl_action_client_t *action_client =
      (rcl_action_client_t *)malloc(sizeof(rcl_action_client_t));
  *action_client = rcl_action_get_zero_initialized_client();
  rcl_action_client_options_t action_client_ops =
      rcl_action_client_get_default_options();

  rcl_ret_t ret =
      rcl_action_client_init(action_client, node, ts, action_name, &action_client_ops);

  *action_client_handle = (void *)action_client;

  return ret;
}

int32_t native_rcl_action_destroy_client_handle(void *action_client_handle, void *node_handle) {
  rcl_action_client_t *action_client = (rcl_action_client_t *)action_client_handle;
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rcl_ret_t ret = rcl_action_client_fini(action_client, node);
  free(action_client);

  return ret;
}

int32_t native_rcl_action_create_server_handle(void **action_server_handle,
                                               void *node_handle,
                                               const char *action_name,
                                               void *typesupport) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rosidl_action_type_support_t *ts =
    (rosidl_action_type_support_t *)typesupport;

  rcl_action_server_t *action_server =
    (rcl_action_server_t *)malloc(sizeof(rcl_action_server_t));
  *action_server = rcl_action_get_zero_initialized_server();
  rcl_action_server_options_t action_server_ops =
    rcl_action_server_get_default_options();

  rcl_clock_t *clock = native_rcl_get_default_clock();

  rcl_ret_t ret =
    rcl_action_server_init(action_server, node, clock, ts, action_name, &action_server_ops);

  *action_server_handle = (void *)action_server;

  return ret;
}

int32_t native_rcl_action_destroy_server_handle(void *action_server_handle, void *node_handle) {
  rcl_action_server_t *action_server = (rcl_action_server_t *)action_server_handle;
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rcl_ret_t ret = rcl_action_server_fini(action_server, node);
  free(action_server);

  return ret;
}

const char * native_rcl_node_get_name_handle(void *node_handle) {
  rcl_node_t *node = (rcl_node_t *)node_handle;
  
  return rcl_node_get_name(node);
}

const char * native_rcl_node_get_namespace_handle(void *node_handle) {
  rcl_node_t *node = (rcl_node_t *)node_handle;
  
  return rcl_node_get_namespace(node);
}

const char * native_rcl_node_get_fully_qualified_name_handle(void *node_handle) {
  rcl_node_t *node = (rcl_node_t *)node_handle;
  
  return rcl_node_get_fully_qualified_name(node);
}