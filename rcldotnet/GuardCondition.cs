/* Copyright 2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
    internal static class GuardConditionDelegates
    {
        internal static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTriggerGuardConditionType(
            SafeGuardConditionHandle guardConditionHandle);

        internal static NativeRCLTriggerGuardConditionType native_rcl_trigger_guard_condition = null;

        static GuardConditionDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            IntPtr native_rcl_trigger_guard_condition_ptr = _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_trigger_guard_condition");
            GuardConditionDelegates.native_rcl_trigger_guard_condition = (NativeRCLTriggerGuardConditionType)Marshal.GetDelegateForFunctionPointer(
                native_rcl_trigger_guard_condition_ptr, typeof(NativeRCLTriggerGuardConditionType));
        }
    }

    public sealed class GuardCondition
    {
        private readonly Action _callback;

        internal GuardCondition(SafeGuardConditionHandle handle, Action callback)
        {
            Handle = handle;
            _callback = callback;
        }

        // GuardCondition does intentionally (for now) not implement IDisposable as this
        // needs some extra consideration how the type works after its
        // internal handle is disposed.
        // By relying on the GC/Finalizer of SafeHandle the handle only gets
        // Disposed if the publisher is not live anymore.
        internal SafeGuardConditionHandle Handle { get; }

        public void Trigger()
        {
            RCLRet ret = GuardConditionDelegates.native_rcl_trigger_guard_condition(Handle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(GuardConditionDelegates.native_rcl_trigger_guard_condition)}() failed.");
        }

        internal void TriggerCallback()
        {
            _callback();
        }
    }
}
