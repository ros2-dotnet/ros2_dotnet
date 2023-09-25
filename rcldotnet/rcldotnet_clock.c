// Copyright 2023 Queensland University of Technology
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

#include "rcldotnet_clock.h"

int32_t native_rcl_clock_get_now(void *clock_handle, int64_t *time_point) {
  rcl_clock_t *clock = (rcl_clock_t *)clock_handle;
  rcl_time_point_value_t *time = (rcl_time_point_value_t *)time_point;
  
  return rcl_clock_get_now(clock, time);
}
