/*
MIT License

Copyright (c) 2021 Microsoft

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>

#include "rosidl_runtime_c/service_type_support_struct.h"

#include "rcldotnet_client.h"

int32_t native_rcl_send_request(void * client_handle, void * raw_ros_message, long * sequence_number)
{
  rcl_client_t * client = (rcl_client_t *)client_handle;

  return rcl_send_request(client, raw_ros_message, sequence_number);
}

int32_t native_rcl_take_response(void * client_handle, void * raw_ros_message)
{
  rmw_service_info_t header;
  rcl_client_t * client = (rcl_client_t *)client_handle;

  return rcl_take_response(client, &header, raw_ros_message);
}

int32_t native_rcl_service_server_is_available(void * client_handle, void * node_handle, bool * is_available)
{
  rcl_client_t * client = (rcl_client_t *)client_handle;
  rcl_node_t * node = (rcl_node_t *)node_handle;

  return rcl_service_server_is_available(node, client, is_available);
}