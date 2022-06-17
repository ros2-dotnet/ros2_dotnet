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

#ifndef RCLDOTNET_H
#define RCLDOTNET_H

#include "rcldotnet_macros.h"

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_init();

RCLDOTNET_EXPORT
const char * RCLDOTNET_CDECL native_rcl_get_rmw_identifier();

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_get_error_string(char *buffer, int32_t bufferSize);

RCLDOTNET_EXPORT
void RCLDOTNET_CDECL native_rcl_reset_error(void);

RCLDOTNET_EXPORT
bool RCLDOTNET_CDECL native_rcl_ok();

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_node_handle(void **, const char *, const char *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_node_handle(void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_guard_condition_handle(void **guard_condition_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_guard_condition_handle(void *guard_condition_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_wait_set_handle(
    void **wait_set_handle,
    int32_t numberOfSubscriptions,
    int32_t numberOfGuardConditions,
    int32_t numberOfTimers,
    int32_t numberOfClients,
    int32_t numberOfServices,
    int32_t numberOfEvents);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_wait_set_handle(void *wait_set_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_clear(void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_subscription(void *, void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_service(void *wait_set_handle, void *service_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_client(void *wait_set_handle, void *client_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait_set_add_guard_condition_handle(void *wait_set_handle, void *guard_condition_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_wait(void *, int64_t);

RCLDOTNET_EXPORT
bool RCLDOTNET_CDECL native_rcl_wait_set_guard_condition_ready(void *wait_set_handle, int32_t index);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_take(void *, void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_request_id_handle(void **request_id_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_request_id_handle(void *request_id_handle);

RCLDOTNET_EXPORT
int64_t RCLDOTNET_CDECL native_rcl_request_id_get_sequence_number(void *request_id_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_take_request(void *service_handle, void *request_header_handle, void *request_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_send_response(void *service_handle, void *request_header_handle, void *resopnse_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_take_response(void *client_handle, void *request_header_handle, void *response_handle);

#endif // RCLDOTNET_H
