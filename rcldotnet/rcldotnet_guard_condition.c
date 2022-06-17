// Copyright 2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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

#include "rcldotnet_guard_condition.h"

int32_t native_rcl_trigger_guard_condition(void *guard_condition_handle)
{
  rcl_guard_condition_t * guard_condition = (rcl_guard_condition_t *)guard_condition_handle;

  rcl_ret_t ret = rcl_trigger_guard_condition(guard_condition);

  return ret;
}
