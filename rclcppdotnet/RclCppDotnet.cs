
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
    internal class RclCppDotnetDelegates {
        internal static readonly DllLoadUtils dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRclcppInitType();
        internal static NativeRclcppInitType native_rclcpp_init = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRclcppShutdownType();
        internal static NativeRclcppShutdownType native_rclcpp_shutdown = null;

        static RclCppDotnetDelegates() {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr pDll = dllLoadUtils.LoadLibrary("rclcppdotnet");

            IntPtr native_rclcpp_init_ptr = dllLoadUtils.GetProcAddress(pDll, "native_rclcpp_init");
            RclCppDotnetDelegates.native_rclcpp_init = (NativeRclcppInitType)Marshal.GetDelegateForFunctionPointer(
                native_rclcpp_init_ptr, typeof(NativeRclcppInitType));

            IntPtr native_rclcpp_shutdown_ptr = dllLoadUtils.GetProcAddress(pDll, "native_rclcpp_shutdown");
            RclCppDotnetDelegates.native_rclcpp_shutdown = (NativeRclcppShutdownType)Marshal.GetDelegateForFunctionPointer(
                native_rclcpp_shutdown_ptr, typeof(NativeRclcppShutdownType));
        }
    }

    public class RclCppDotnet {
        private static bool initialized = false;
        private static readonly object syncLock = new object();

        public static void Init()
        {
            lock (syncLock)
            {
                if (!initialized)
                {
                    RclCppDotnetDelegates.native_rclcpp_init();
                    initialized = true;
                }
            }
        }

        public static void Shutdown()
        {
            lock (syncLock)
            {
                if (initialized)
                {
                    RclCppDotnetDelegates.native_rclcpp_shutdown();
                    initialized = false;
                }
            }
        }
    }
}