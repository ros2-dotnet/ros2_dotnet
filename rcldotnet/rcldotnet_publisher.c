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
#include <stdio.h>

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rcldotnet_publisher.h"

void native_rcl_publish(void * publisher_handle, void * raw_ros_message)
{
  rcl_publisher_t * publisher = (rcl_publisher_t *)publisher_handle;

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message, NULL);

  // TODO(esteve): handle error
  if (ret != RCL_RET_OK) {
    assert(false);
  }
}
