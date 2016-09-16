// Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rosidl_generator_c/message_type_support.h>

#include "rcldotnet.h"

bool native_rcl_ok()
{
  return rcl_ok();
}

void native_rcl_init()
{
  rcl_ret_t ret = rcl_init(0, NULL, rcl_get_default_allocator());
  // TODO(esteve): check return status
  if (ret != RCL_RET_OK)
  {
    assert(false);
  }
}

void * native_rcl_create_node_handle(const char * node_name)
{
  rcl_node_t * node = (rcl_node_t *)malloc(sizeof(rcl_node_t));
  node->impl = NULL;
  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, node_name, &default_options);
  if (ret != RCL_RET_OK)
  {
    assert(false);

    return NULL;
  }

  return node;
}

const char * native_rcl_get_rmw_identifier()
{
  return rmw_get_implementation_identifier();
}
