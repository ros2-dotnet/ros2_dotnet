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
        internal delegate RCLRet NativeRCLTimerCallType(SafeTimerHandle timerHandle);

        internal static NativeRCLTimerCallType native_rcl_timer_call = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTimerCancelType(SafeTimerHandle timerHandle);
        internal static NativeRCLTimerCancelType native_rcl_timer_cancel = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTimerResetType(SafeTimerHandle timerHandle);
        internal static NativeRCLTimerResetType native_rcl_timer_reset = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTimerIsCanceledType(SafeTimerHandle timerHandle, out int isCanceled);
        internal static NativeRCLTimerIsCanceledType native_rcl_timer_is_canceled = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTimerIsReadyType(SafeTimerHandle timerHandle, out int isReady);
        internal static NativeRCLTimerIsReadyType native_rcl_timer_is_ready = null;

        static TimerDelegates()
        {
            var dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = dllLoadUtils.LoadLibrary("rcldotnet");

            dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_timer_call), out native_rcl_timer_call);
            dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_timer_cancel), out native_rcl_timer_cancel);
            dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_timer_reset), out native_rcl_timer_reset);
            dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_timer_is_canceled), out native_rcl_timer_is_canceled);
            dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_timer_is_ready), out native_rcl_timer_is_ready);
        }
    }

    public sealed class Timer
    {
        private readonly Action<TimeSpan> _callback;

        // The garbage collector will eventually try to clean up the delegate if nothing on the .NET side is holding on to it.
        private readonly TimerCallback _internalCallback;

        internal Timer(Clock clock, TimeSpan period, Action<TimeSpan> callback)
        {
            _callback = callback;

            SafeTimerHandle handle = new SafeTimerHandle();
            _internalCallback = OnTimer;
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_timer_handle(ref handle, clock.Handle, new Duration(period), _internalCallback);

            if (ret != RCLRet.Ok)
            {
                handle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_timer_handle)}() failed.");
            }

            Handle = handle;
        }

        public void Cancel()
        {
            RCLRet ret = TimerDelegates.native_rcl_timer_cancel(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_cancel)}() failed.");
        }

        public void Reset()
        {
            RCLRet ret = TimerDelegates.native_rcl_timer_reset(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_reset)}() failed.");
        }

        public bool IsCanceled
        {
            get
            {
                RCLRet ret = TimerDelegates.native_rcl_timer_is_canceled(Handle, out int isCanceled);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_is_canceled)}() failed.");
                return isCanceled != 0;
            }
        }

        public bool IsReady
        {
            get
            {
                RCLRet ret = TimerDelegates.native_rcl_timer_is_ready(Handle, out int isReady);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_is_ready)}() failed.");
                return isReady != 0;
            }
        }

        internal SafeTimerHandle Handle { get; }

        internal void Call()
        {
            RCLRet ret = TimerDelegates.native_rcl_timer_call(Handle);
            if (ret == ROS2.RCLRet.TimerCanceled)
            {
                // Timer was canceled, do nothing.
                return;
            }
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(TimerDelegates.native_rcl_timer_call)}() failed.");
        }

        private void OnTimer(IntPtr handle, Duration elapsed)
        {
            _callback?.Invoke(elapsed.AsTimespan());
        }
    }
}
