// Copyright 2021 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
#include <stdio.h>

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rcl/client.h>
#include <rcl/graph.h>

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rcldotnet_client.h"

int32_t native_rcl_send_request(void *client_handle, void *request_handle, int64_t *sequence_number) {
  rcl_client_t * client = (rcl_client_t *)client_handle;

  rcl_ret_t ret = rcl_send_request(client, request_handle, sequence_number);
  return ret;
}

int32_t native_rcl_service_server_is_available(void *node_handle, void *client_handle, bool *is_available) {
  rcl_node_t * node = (rcl_node_t *)node_handle;
  rcl_client_t * client = (rcl_client_t *)client_handle;

  rcl_ret_t ret = rcl_service_server_is_available(node, client, is_available);
  return ret;
}
