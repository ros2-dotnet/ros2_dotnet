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
        private readonly Action<Duration> _callback;

        // The garbage collector will eventually try to clean up the delegate if nothing on the .NET side is holding on to it.
        private readonly TimerCallback _internalCallback;

        internal Timer(Clock clock, Duration period, Action<Duration> callback)
        {
            _callback = callback;

            SafeTimerHandle handle = new SafeTimerHandle();
            _internalCallback = OnTimer;
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_timer_handle(ref handle, clock.Handle, period, _internalCallback);

            if (ret != RCLRet.Ok)
            {
                handle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_timer_handle)}() failed.");
            }

            Handle = handle;
        }

        internal SafeTimerHandle Handle { get; }

        internal void Call()
        {
            RCLRet ret = TimerDelegates.native_rcl_timer_call(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_call)}() failed.");
        }

        private void OnTimer(IntPtr handle, Duration elapsed)
        {
            _callback?.Invoke(elapsed);
        }
    }
}
