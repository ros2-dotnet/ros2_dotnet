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

#ifndef RCLDOTNET_PARAMS_H
#define RCLDOTNET_PARAMS_H

#include "rcldotnet_macros.h"

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_destroy_rcl_params(void *rcl_params);

RCLDOTNET_EXPORT
int32_t /* bool */ RCLDOTNET_CDECL native_rcl_try_get_parameter(void *param_value_handle, const void *params_handle, const void *node_handle, const char *name);

#endif // RCLDOTNET_PARAMS_H
