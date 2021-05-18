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

#ifndef RCLCPPDOTNET_MACROS_H
#define RCLCPPDOTNET_MACROS_H

#if defined(_MSC_VER)
    #define RCLCPPDOTNET_EXPORT __declspec(dllexport)
    #define RCLCPPDOTNET_IMPORT __declspec(dllimport)
    #define RCLCPPDOTNET_CDECL __cdecl
#elif defined(__GNUC__)
    #define RCLCPPDOTNET_EXPORT __attribute__((visibility("default")))
    #define RCLCPPDOTNET_IMPORT
    #define RCLCPPDOTNET_CDECL
#else
    #define RCLCPPDOTNET_EXPORT
    #define RCLCPPDOTNET_IMPORT
    #define RCLCPPDOTNET_CDECL
    #pragma warning Unknown dynamic link import/export semantics.
#endif

#endif // RCLCPPDOTNET_MACROS_H
