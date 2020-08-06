
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
    internal class TransformListenerDelegates
    {
        internal static readonly DllLoadUtils dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeConstructBufferType();
        internal static NativeConstructBufferType native_construct_buffer = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeDeleteBufferType(IntPtr buf);
        internal static NativeDeleteBufferType native_delete_buffer = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeConstructListenerType(IntPtr buf);
        internal static NativeConstructListenerType native_construct_listener = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeDeleteListenerType(IntPtr listener);
        internal static NativeDeleteListenerType native_delete_listener = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeConstructTimeType(int sec, int nano);
        internal static NativeConstructTimeType native_construct_time = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeLookupTransformType(IntPtr buf, string from, string to, IntPtr t);
        internal static NativeLookupTransformType native_lookup_transform = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate double NativeRetrieveTranslationType(IntPtr tf, out TfVector3 vec);
        internal static NativeRetrieveTranslationType native_retrieve_translation = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate double NativeRetrieveRotationType(IntPtr tf, out TfQuaternion vec);
        internal static NativeRetrieveRotationType native_retrieve_rotation = null;

        static TransformListenerDelegates()
        {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibTFL = dllLoadUtils.LoadLibrary("transform_listener");

            IntPtr native_construct_buffer_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_construct_buffer");
            TransformListenerDelegates.native_construct_buffer = (NativeConstructBufferType)Marshal.GetDelegateForFunctionPointer(
                native_construct_buffer_ptr, typeof(NativeConstructBufferType));

            IntPtr native_delete_buffer_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_delete_buffer");
            TransformListenerDelegates.native_delete_buffer = (NativeDeleteBufferType)Marshal.GetDelegateForFunctionPointer(
                native_delete_buffer_ptr, typeof(NativeDeleteBufferType));

            IntPtr native_construct_listener_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_construct_listener");
            TransformListenerDelegates.native_construct_listener = (NativeConstructListenerType)Marshal.GetDelegateForFunctionPointer(
                native_construct_listener_ptr, typeof(NativeConstructListenerType));

            IntPtr native_delete_listener_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_delete_listener");
            TransformListenerDelegates.native_delete_listener = (NativeDeleteListenerType)Marshal.GetDelegateForFunctionPointer(
                native_delete_listener_ptr, typeof(NativeDeleteListenerType));

            IntPtr native_construct_time_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_construct_time");
            TransformListenerDelegates.native_construct_time = (NativeConstructTimeType)Marshal.GetDelegateForFunctionPointer(
                native_construct_time_ptr, typeof(NativeConstructTimeType));

            IntPtr native_lookup_transform_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_lookup_transform");
            TransformListenerDelegates.native_lookup_transform = (NativeLookupTransformType)Marshal.GetDelegateForFunctionPointer(
                native_lookup_transform_ptr, typeof(NativeLookupTransformType));

            IntPtr native_retrieve_translation_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_retrieve_translation");
            TransformListenerDelegates.native_retrieve_translation = (NativeRetrieveTranslationType)Marshal.GetDelegateForFunctionPointer(
                native_retrieve_translation_ptr, typeof(NativeRetrieveTranslationType));

            IntPtr native_retrieve_rotation_ptr = dllLoadUtils.GetProcAddress(nativeLibTFL, "native_retrieve_rotation");
            TransformListenerDelegates.native_retrieve_rotation = (NativeRetrieveRotationType)Marshal.GetDelegateForFunctionPointer(
                native_retrieve_rotation_ptr, typeof(NativeRetrieveRotationType));
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TfVector3
    {
        public double x;
        public double y;
        public double z;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TfQuaternion
    {
        public double x;
        public double y;
        public double z;
        public double w;
    }

    /// <summary>
    /// Grants access to the TransformListener, a node on the ROS2 network
    /// that manages the /tf and /tf_static subscriptions.
    /// All TransformListener instances in .NET share a single node, created 
    /// upon construction of the first TransformListener.
    /// </summary>
    public class TransformListener {
        private static IntPtr buf = IntPtr.Zero;
        private static IntPtr listener = IntPtr.Zero;

        public TransformListener()
        {
            if (buf == IntPtr.Zero)
            {
                buf = TransformListenerDelegates.native_construct_buffer();
            }

            if (listener == IntPtr.Zero)
            {
                listener = TransformListenerDelegates.native_construct_listener(buf);
            }
        }

        ~TransformListener()
        {
            if (buf != IntPtr.Zero)
            {
                TransformListenerDelegates.native_delete_buffer(buf);
                buf = IntPtr.Zero;
            }

            if (listener != IntPtr.Zero)
            {
                TransformListenerDelegates.native_delete_listener(listener);
                listener = IntPtr.Zero;
            }
        }

        /// <summary>
        /// Returns the ROS2 translation between frames 'from' and 'to', or null
        /// if one or more of the frames do not exist.
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public TfVector3? LookupTranslation(string from, string to) 
        {
            IntPtr transformHandle = IntPtr.Zero;

            transformHandle = TransformListenerDelegates.native_lookup_transform(buf,
                from, to, TransformListenerDelegates.native_construct_time(0, 0));

            if (transformHandle == IntPtr.Zero)
            {
                return null;
            }

            TfVector3 output = new TfVector3();
            TransformListenerDelegates.native_retrieve_translation(transformHandle, out output);
            return output;
        }

        /// <summary>
        /// Returns the ROS2 rotation between frames 'from' and 'to', or null
        /// if one or more of the frames do not exist.
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public TfQuaternion? LookupRotation(string from, string to)
        {
            IntPtr transformHandle = IntPtr.Zero;

            transformHandle = TransformListenerDelegates.native_lookup_transform(buf,
                from, to, TransformListenerDelegates.native_construct_time(0, 0));

            if (transformHandle == IntPtr.Zero)
            {
                return null;
            }

            TfQuaternion output = new TfQuaternion();
            TransformListenerDelegates.native_retrieve_rotation(transformHandle, out output);
            return output;
        }
    }
}