/* Copyright 2023 Queensland University of Technology.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Runtime.InteropServices;
using rcl_interfaces.msg;
using ROS2.Utils;

namespace ROS2.ParameterHandling {
    internal static class ParameterDelegates
    {
        private static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRCLDestroyRclParamsType(IntPtr paramsHandle);

        internal static NativeRCLDestroyRclParamsType native_rcl_destroy_rcl_params = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal delegate int NativeRCLTryGetParameterType(SafeHandle parameterValueHandle, SafeRclParamsHandle paramsHandle, SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string name);

        internal static NativeRCLTryGetParameterType native_rcl_try_get_parameter = null;

        static ParameterDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_destroy_rcl_params), out native_rcl_destroy_rcl_params);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_try_get_parameter), out native_rcl_try_get_parameter);
        }
    }
}
