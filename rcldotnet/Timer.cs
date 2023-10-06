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
using ROS2.Utils;

namespace ROS2
{
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void TimerCallback(IntPtr timer, Duration elapsed);

    internal static class TimerDelegates
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTimerFunctionType(SafeTimerHandle timerHandle);

        internal static NativeRCLTimerFunctionType native_rcl_timer_call = null;

        static TimerDelegates()
        {
            var dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = dllLoadUtils.LoadLibrary("rcldotnet");

            dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_timer_call), out native_rcl_timer_call);
        }
    }

    public sealed class Timer
    {
        // The garbage collector will eventually try to clean up the delegate if nothing on the .NET side is holding on to it.
        private readonly TimerCallback _callback;

        internal Timer(SafeTimerHandle handle, TimerCallback callback)
        {
            Handle = handle;
            _callback = callback;
        }

        internal SafeTimerHandle Handle { get; }


        internal void Call()
        {
            RCLRet ret = TimerDelegates.native_rcl_timer_call(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_call)}() failed.");
        }
    }
}
