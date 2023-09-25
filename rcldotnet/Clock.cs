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
    /// <summary>
    /// Time source type, used to indicate the source of a time measurement.
    ///
    /// ROSTime will report the latest value reported by a ROS time source, or
    /// if a ROS time source is not active it reports the same as RCL_SYSTEM_TIME.
    /// For more information about ROS time sources, refer to the design document:
    /// http://design.ros2.org/articles/clock_and_time.html
    ///
    /// SystemTime reports the same value as the system clock.
    ///
    /// SteadyTime reports a value from a monotonically increasing clock.
    /// </summary>
    public enum ClockType
    {
        ClockUninitialized = 0,
        ROSTime,
        SystemTime,
        SteadyTime
    }

    public struct Time
    {
        public uint seconds;
        public uint nanoseconds;
    }

    internal static class ClockDelegates
    {
        private static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLClockGetNowType(
            SafeClockHandle clockHandle, out long time);

        internal static NativeRCLClockGetNowType native_rcl_clock_get_now = null;

        static ClockDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            IntPtr native_rcl_clock_get_now_ptr = _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_clock_get_now");
            ClockDelegates.native_rcl_clock_get_now = (NativeRCLClockGetNowType)Marshal.GetDelegateForFunctionPointer(
                native_rcl_clock_get_now_ptr, typeof(NativeRCLClockGetNowType));
        }
    }

    public sealed class Clock
    {
        private const long SECONDS_TO_NANOSECONDS = 1000L * 1000L * 1000L;

        internal Clock(SafeClockHandle handle)
        {
            Handle = handle;
        }

        internal SafeClockHandle Handle { get; }

        public Time Now()
        {
            RCLRet ret = ClockDelegates.native_rcl_clock_get_now(Handle, out long timePoint);

            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_clock_get_now)}() failed.");

            Time time = new Time();
            time.seconds = (uint)(timePoint / SECONDS_TO_NANOSECONDS);
            time.nanoseconds = (uint)(timePoint - (time.seconds * SECONDS_TO_NANOSECONDS));

            return time;
        }
    }
}