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

#ifndef RCLDOTNET_NODE_H
#define RCLDOTNET_NODE_H

#include "rcldotnet_macros.h"

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_publisher_handle(void **, void *,
                                                           const char *,
                                                           void *,
                                                           void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_publisher_handle(void *publisher_handle, void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_subscription_handle(void **, void *,
                                                              const char *,
                                                              void *,
                                                              void *);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_subscription_handle(void *subscription_handle, void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_service_handle(void **service_handle,
                                                         void *node_handle,
                                                         const char *service_name,
                                                         void *typesupport);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_service_handle(void *service_handle, void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_create_client_handle(void **client_handle,
                                                        void *node_handle,
                                                        const char *service_name,
                                                        void *typesupport);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_destroy_client_handle(void *client_handle, void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_create_client_handle(void **action_client_handle,
                                                               void *node_handle,
                                                               const char *action_name,
                                                               void *typesupport);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_destroy_client_handle(void *action_client_handle, void *node_handle);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_create_server_handle(void **action_server_handle,
                                                               void *node_handle,
                                                               void *clock_handle,
                                                               const char *action_name,
                                                               void *typesupport);

RCLDOTNET_EXPORT
int32_t RCLDOTNET_CDECL native_rcl_action_destroy_server_handle(void *action_server_handle, void *node_handle);

RCLDOTNET_EXPORT
const char * RCLDOTNET_CDECL native_rcl_node_get_name_handle(void *node_handle);

RCLDOTNET_EXPORT
const char * RCLDOTNET_CDECL native_rcl_node_get_namespace_handle(void *node_handle);

RCLDOTNET_EXPORT
const char * RCLDOTNET_CDECL native_rcl_node_get_fully_qualified_name_handle(void *node_handle);

#endif // RCLDOTNET_NODE_H
