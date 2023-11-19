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
using System.Collections.Generic;
using System.Runtime.InteropServices;
using builtin_interfaces.msg;
using ROS2.Utils;

namespace ROS2
{
    internal static class TimeConstants
    {
        public const long NanosecondsPerSecond = 1000L * 1000L * 1000L;
        public const long NanosecondsPerMillisecond = 1000000L;

        public const long NanosecondsPerTimespanTick = NanosecondsPerMillisecond / TimeSpan.TicksPerMillisecond; // ~= 100.
    }

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
        ROSTime = 1,
        SystemTime = 2,
        SteadyTime = 3
    }

    /// Enumeration to describe the type of time jump.
    /// see definition here: https://github.com/ros2/rcl/blob/master/rcl/include/rcl/time.h
    public enum ClockChange
    {
        /// The source before and after the jump is ROS_TIME.
        RosTimeNoChange = 1,
        /// The source switched to ROS_TIME from SYSTEM_TIME.
        RosTimeActivated = 2,
        /// The source switched to SYSTEM_TIME from ROS_TIME.
        RosTimeDeactivated = 3,
        /// The source before and after the jump is SYSTEM_TIME.
        SystemTimeNoChange = 4
    }

    // Internal as TimeSpan should be user-facing.
    [StructLayout(LayoutKind.Sequential)]
    internal readonly struct Duration
    {
        private readonly long _nanoseconds;

        public Duration(TimeSpan timeSpan)
        {
            _nanoseconds = checked(timeSpan.Ticks * TimeConstants.NanosecondsPerTimespanTick);
        }

        public TimeSpan AsTimespan()
        {
            return new TimeSpan(_nanoseconds / TimeConstants.NanosecondsPerTimespanTick);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public readonly struct TimeJump
    {
        private readonly ClockChange _clockChange;
        private readonly Duration _delta;

        internal TimeJump(ClockChange clockChange, Duration delta)
        {
            _clockChange = clockChange;
            _delta = delta;
        }

        public ClockChange ClockChange => _clockChange;
        public TimeSpan Delta => _delta.AsTimespan();
    }

    internal delegate void JumpCallbackInternal(IntPtr timeJumpPtr, bool beforeJump);

    public delegate void PreJumpCallback();
    public delegate void PostJumpCallback(TimeJump timeJump);

    [StructLayout(LayoutKind.Sequential)]
    public readonly struct JumpThreshold
    {
        internal readonly bool _onClockChange;
        internal readonly Duration _minForward;
        internal readonly Duration _minBackward;

        public JumpThreshold(bool onClockChange, TimeSpan minForward, TimeSpan minBackward)
        {
            _onClockChange = onClockChange;
            _minForward = new Duration(minForward);
            _minBackward = new Duration(minBackward);
        }

        public bool OnClockChange => _onClockChange;
        public TimeSpan MinForward => _minForward.AsTimespan();
        public TimeSpan MinBackward => _minBackward.AsTimespan();
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct TimePoint
    {

        internal long nanoseconds;

        internal Time ToMsg()
        {
            long sec = nanoseconds / TimeConstants.NanosecondsPerSecond;
            long nanosec = nanoseconds - (sec * TimeConstants.NanosecondsPerSecond);
            return new Time
            {
                Sec = (int)sec,
                Nanosec = (uint)nanosec
            };
        }

        internal static TimePoint FromMsg(Time message)
        {
            return new TimePoint
            {
                nanoseconds = message.Sec * TimeConstants.NanosecondsPerSecond + message.Nanosec
            };
        }
    }

    public class CallbackAlreadyRegisteredException : Exception
    {
        public CallbackAlreadyRegisteredException(string message) : base(message)
        {
        }
    }

    internal static class ClockDelegates
    {
        private static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLClockFunctionType(SafeClockHandle clockHandle);

        internal static NativeRCLClockFunctionType native_rcl_enable_ros_time_override = null;

        internal static NativeRCLClockFunctionType native_rcl_disable_ros_time_override = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLClockGetNowType(SafeClockHandle clockHandle, out TimePoint time);

        internal static NativeRCLClockGetNowType native_rcl_clock_get_now = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLSetRosTimeOverrideType(SafeClockHandle clockHandle, long timePointValue);

        internal static NativeRCLSetRosTimeOverrideType native_rcl_set_ros_time_override = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLAddJumpCallbackType(SafeClockHandle clockHandle, JumpThreshold threshold, JumpCallbackInternal callback);

        internal static NativeRCLAddJumpCallbackType native_rcl_clock_add_jump_callback = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLRemoveJumpCallbackType(SafeClockHandle clockHandle, JumpCallbackInternal callback);

        internal static NativeRCLRemoveJumpCallbackType native_rcl_clock_remove_jump_callback = null;

        static ClockDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_enable_ros_time_override), out native_rcl_enable_ros_time_override);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_disable_ros_time_override), out native_rcl_disable_ros_time_override);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_clock_get_now), out native_rcl_clock_get_now);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_set_ros_time_override), out native_rcl_set_ros_time_override);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_clock_add_jump_callback), out native_rcl_clock_add_jump_callback);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_clock_remove_jump_callback), out native_rcl_clock_remove_jump_callback);
        }
    }

    public sealed class Clock
    {
        private readonly HashSet<JumpCallbackInternal> _registeredJumpCallbacks = new HashSet<JumpCallbackInternal>();

        private readonly object _lock = new object();

        internal Clock(SafeClockHandle handle)
        {
            Handle = handle;
        }

        internal SafeClockHandle Handle { get; }

        public Time Now()
        {
            RCLRet ret = ClockDelegates.native_rcl_clock_get_now(Handle, out TimePoint timePoint);

            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_clock_get_now)}() failed.");

            return timePoint.ToMsg();
        }

        internal RCLRet EnableRosTimeOverride()
        {
            RCLRet ret;
            lock (_lock)
            {
                ret = ClockDelegates.native_rcl_enable_ros_time_override(Handle);
            }

            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_enable_ros_time_override)}() failed.");

            return ret;
        }

        internal RCLRet DisableRosTimeOverride()
        {
            RCLRet ret;
            lock (_lock)
            {
                ret = ClockDelegates.native_rcl_disable_ros_time_override(Handle);
            }

            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_disable_ros_time_override)}() failed.");

            return ret;
        }

        internal RCLRet SetRosTimeOverride(long timePointValue)
        {
            RCLRet ret;
            lock (_lock)
            {
                ret = ClockDelegates.native_rcl_set_ros_time_override(Handle, timePointValue);
            }

            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_set_ros_time_override)}() failed.");

            return ret;
        }

        /// <summary>
        /// Add a callback to be called when a time jump exceeds a threshold.
        /// </summary>
        /// <param name="threshold">Callbacks will be triggered if the time jump is greater then the threshold.</param>
        /// <param name="preJumpCallback">Callback to be called before new time is set.</param>
        /// <param name="postJumpCallback">Callback to be called after new time is set.</param>
        /// <returns>A disposable object that can be used to remove the callbacks from the clock.</returns>
        public IDisposable CreateJumpCallback(JumpThreshold threshold, PreJumpCallback preJumpCallback, PostJumpCallback postJumpCallback)
        {
            JumpHandler jumpHandler = new JumpHandler(this, preJumpCallback, postJumpCallback);

            lock (_lock)
            {
                RCLRet ret = ClockDelegates.native_rcl_clock_add_jump_callback(Handle, threshold, jumpHandler.JumpCallback);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_clock_add_jump_callback)}() failed.");

                // Save a reference to the JumpCallback that was passed down to
                // the native side. The PInvoke interop does create a stub that
                // get's passed down so that the delegate can be called via a
                // native function pointer. This stub does not get reallocated
                // by the CG, so no extra pinning is needed, but it will get
                // deallocated once the delegate object gets collected. So we
                // need to make shure to have a strong reference to the
                // delegate, otherwise the native side would call a function
                // pointer that points to the deallocated stub.
                // rcl_clock_add_jump_callback only adds the provided callback
                // to it's list if it returned OK, so we don't need to worry
                // about references to the delegate from the native side in this
                // case.
                _registeredJumpCallbacks.Add(jumpHandler.JumpCallback);
            }

            return jumpHandler;
        }

        internal void RemoveJumpCallback(JumpHandler jumpHandler, ref bool jumpHandlerDisposed)
        {
            lock (_lock)
            {
                if (jumpHandlerDisposed)
                    return;

                RCLRet ret = ClockDelegates.native_rcl_clock_remove_jump_callback(Handle, jumpHandler.JumpCallback);

                // Calling Dispose multiple times should not throw errors.
                // rcl_clock_remove_jump_callback failues:
                // - The null-checks can't fail as it is ensured that we always
                //   pass valid object references down.
                // - We track if a JumpHandler is disposed via a flag, so the
                //   "jump callback was not found" error can't happen.
                // - Only failues for allocation and internal errors in
                //   rcldotnet could cause exceptions.
                //
                // So we should be save to throw exceptions here, the user of
                // the library should have no way to trigger it. 
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClockDelegates.native_rcl_clock_remove_jump_callback)}() failed.");

                jumpHandlerDisposed = true;
                _registeredJumpCallbacks.Remove(jumpHandler.JumpCallback);
            }
        }
    }

    internal sealed class JumpHandler : IDisposable
    {
        private readonly Clock _clock;
        private readonly PreJumpCallback _preJumpCallback;
        private readonly PostJumpCallback _postJumpCallback;

        private bool _disposed;

        public JumpHandler(Clock clock, PreJumpCallback preJumpCallback, PostJumpCallback postJumpCallback)
        {
            _clock = clock;
            _preJumpCallback = preJumpCallback;
            _postJumpCallback = postJumpCallback;

            // Only create the delegate once and cache in instance
            // so the same instance is avaliable to deregister.
            JumpCallback = new JumpCallbackInternal(OnJump);
        }

        internal JumpCallbackInternal JumpCallback { get; }

        public void Dispose()
        {
            // fast return without look
            if (_disposed)
                return;

            _clock.RemoveJumpCallback(this, ref _disposed);
        }

        private void OnJump(IntPtr timeJumpPtr, bool beforeJump)
        {
            try
            {
                if (beforeJump)
                {
                    _preJumpCallback?.Invoke();
                }
                else
                {
                    _postJumpCallback?.Invoke(Marshal.PtrToStructure<TimeJump>(timeJumpPtr));
                }
            }
            catch
            {
                // Catch all exceptions, as on non-windows plattforms exceptions
                // that are propageted to native code can cause crashes.
                // see https://learn.microsoft.com/en-us/dotnet/standard/native-interop/exceptions-interoperability

                // TODO: (sh) Add error handling/logging.
            }
        }
    }
}
