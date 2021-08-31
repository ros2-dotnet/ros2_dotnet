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

#ifndef RCLDOTNET_CLIENT_H
#define RCLDOTNET_CLIENT_H

#include "rcldotnet_macros.h"

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_send_request(void *client_handle, void *request_handle, int64_t *sequence_number);

#endif // RCLDOTNET_CLIENT_H
