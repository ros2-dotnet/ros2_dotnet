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

#include <assert.h>
#include <stdlib.h>

#include <rcl/rcl.h>

#include "rcldotnet_timer.h"

int32_t native_rcl_timer_call(void *timer_handle) {
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;

  return rcl_timer_call(timer);
}

int32_t native_rcl_timer_cancel(void *timer_handle) {
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;

  return rcl_timer_cancel(timer);
}

int32_t native_rcl_timer_reset(void *timer_handle) {
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;

  return rcl_timer_reset(timer);
}

int32_t native_rcl_timer_is_canceled(void *timer_handle, int32_t *is_canceled) {
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;

  bool is_canceled_as_bool;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled_as_bool);
  *is_canceled = is_canceled_as_bool ? 1 : 0;
  return ret;
}

int32_t native_rcl_timer_is_ready(void *timer_handle, int32_t *is_ready) {
  rcl_timer_t *timer = (rcl_timer_t *)timer_handle;

  bool is_ready_as_bool;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready_as_bool);
  *is_ready = is_ready_as_bool ? 1 : 0;
  return ret;
}
