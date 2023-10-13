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

namespace ROS2.Qos
{
    internal struct RmwTime
    {
        public ulong sec;
        public ulong nsec;

        public RmwTime(QosProfileDelegates.NativeRCLQosProfileReadRMWTimeType nativeDelegate, IntPtr profile)
        {
            sec = 0UL;
            nsec = 0UL;

            nativeDelegate(profile, ref sec, ref nsec);
        }

        public TimeSpan AsTimespan()
        {
            if (sec == 9223372036UL && nsec == 854775807UL)
            {
                // see RMW_DURATION_INFINITE and comment on QosProfile.InfiniteDuration above.
                return QosProfile.InfiniteDuration;
            }

            const ulong NanosecondsPerTick = 1000000 / TimeSpan.TicksPerMillisecond; // ~= 100.
            ulong ticks = sec * TimeSpan.TicksPerSecond;
            ticks += nsec / NanosecondsPerTick;
            return new TimeSpan((long)ticks);
        }
    }

    internal static class QosProfileDelegates
    {
        private static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeRCLGetConstQosProfileHandleType();
        internal static NativeRCLGetConstQosProfileHandleType native_rcl_qos_get_const_profile_default = null;
        internal static NativeRCLGetConstQosProfileHandleType native_rcl_qos_get_const_profile_parameter_events = null;
        internal static NativeRCLGetConstQosProfileHandleType native_rcl_qos_get_const_profile_parameters = null;
        internal static NativeRCLGetConstQosProfileHandleType native_rcl_qos_get_const_profile_sensor_data = null;
        internal static NativeRCLGetConstQosProfileHandleType native_rcl_qos_get_const_profile_services_default = null;
        internal static NativeRCLGetConstQosProfileHandleType native_rcl_qos_get_const_profile_system_default = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate QosHistoryPolicy NativeRCLQosProfileReadHistoryType(IntPtr qosProfileHandle);
        internal static NativeRCLQosProfileReadHistoryType native_rcl_qos_profile_read_history;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLQosProfileReadDepthType(IntPtr qosProfileHandle);
        internal static NativeRCLQosProfileReadDepthType native_rcl_qos_profile_read_depth;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate QosReliabilityPolicy NativeRCLQosProfileReadReliabilityType(IntPtr qosProfileHandle);
        internal static NativeRCLQosProfileReadReliabilityType native_rcl_qos_profile_read_reliability;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate QosDurabilityPolicy NativeRCLQosProfileReadDurabilityType(IntPtr qosProfileHandle);
        internal static NativeRCLQosProfileReadDurabilityType native_rcl_qos_profile_read_durability;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRCLQosProfileReadRMWTimeType(IntPtr qosProfileHandle, ref ulong sec, ref ulong nsec);
        internal static NativeRCLQosProfileReadRMWTimeType native_rcl_qos_profile_read_deadline;
        internal static NativeRCLQosProfileReadRMWTimeType native_rcl_qos_profile_read_lifespan;
        internal static NativeRCLQosProfileReadRMWTimeType native_rcl_qos_profile_read_liveliness_lease_duration;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate QosLivelinessPolicy NativeRCLQosProfileReadLivelinessType(IntPtr qosProfileHandle);
        internal static NativeRCLQosProfileReadLivelinessType native_rcl_qos_profile_read_liveliness;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int NativeRCLQosProfileReadAvoidRosNamespaceConventionsType(IntPtr qosProfileHandle);
        internal static NativeRCLQosProfileReadAvoidRosNamespaceConventionsType native_rcl_qos_profile_read_avoid_ros_namespace_conventions;

        static QosProfileDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_get_const_profile_default), out native_rcl_qos_get_const_profile_default);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_get_const_profile_parameter_events), out native_rcl_qos_get_const_profile_parameter_events);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_get_const_profile_parameters), out native_rcl_qos_get_const_profile_parameters);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_get_const_profile_sensor_data), out native_rcl_qos_get_const_profile_sensor_data);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_get_const_profile_services_default), out native_rcl_qos_get_const_profile_services_default);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_get_const_profile_system_default), out native_rcl_qos_get_const_profile_system_default);

            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_history), out native_rcl_qos_profile_read_history);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_depth), out native_rcl_qos_profile_read_depth);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_reliability), out native_rcl_qos_profile_read_reliability);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_durability), out native_rcl_qos_profile_read_durability);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_deadline), out native_rcl_qos_profile_read_deadline);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_lifespan), out native_rcl_qos_profile_read_lifespan);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_liveliness), out native_rcl_qos_profile_read_liveliness);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_liveliness_lease_duration), out native_rcl_qos_profile_read_liveliness_lease_duration);
            _dllLoadUtils.RegisterNativeFunction(nativeLibrary, nameof(native_rcl_qos_profile_read_avoid_ros_namespace_conventions), out native_rcl_qos_profile_read_avoid_ros_namespace_conventions);
        }
    }
}
