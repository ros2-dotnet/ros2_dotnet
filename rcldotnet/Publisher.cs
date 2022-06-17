/* Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
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
    internal static class PublisherDelegates
    {
        internal static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLPublishType(
            SafePublisherHandle publisherHandle, SafeHandle messageHandle);

        internal static NativeRCLPublishType native_rcl_publish = null;

        static PublisherDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            IntPtr native_rcl_publish_ptr = _dllLoadUtils.GetProcAddress(nativeLibrary, "native_rcl_publish");
            PublisherDelegates.native_rcl_publish = (NativeRCLPublishType)Marshal.GetDelegateForFunctionPointer(
                native_rcl_publish_ptr, typeof(NativeRCLPublishType));
        }
    }

    /// <summary>
    /// Base class of a Publisher without generic type arguments for use in collections or so.
    /// </summary>
    public abstract class Publisher
    {
        // Only allow internal subclasses.
        internal Publisher()
        {
        }

        // Publisher does intentionaly (for now) not implement IDisposable as this
        // needs some extra consideration how the type works after its
        // internal handle is disposed.
        // By relying on the GC/Finalizer of SafeHandle the handle only gets
        // Disposed if the publisher is not live anymore.
        internal abstract SafePublisherHandle Handle { get; }
    }

    public sealed class Publisher<T> : Publisher
        where T : IRosMessage
    {

        internal Publisher(SafePublisherHandle handle)
        {
            Handle = handle;
        }

        internal override SafePublisherHandle Handle { get; }

        public void Publish(T message)
        {
            using (var messageHandle = MessageStaticMemberCache<T>.CreateMessageHandle())
            {
                bool mustRelease = false;
                try
                {
                    // Using SafeHandles for __WriteToHandle() is very tedious as this needs to be
                    // handled in generated code across multiple assemblies.
                    // Array and collection indexing would need to create SafeHandles everywere.
                    // It's not worth it, especialy considering the extra allocations for SafeHandles in
                    // arrays or collections that don't realy represent their own native recource.
                    messageHandle.DangerousAddRef(ref mustRelease);
                    message.__WriteToHandle(messageHandle.DangerousGetHandle());
                }
                finally
                {
                    if (mustRelease)
                    {
                        messageHandle.DangerousRelease();
                    }
                }

                RCLRet ret = PublisherDelegates.native_rcl_publish(Handle, messageHandle);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(PublisherDelegates.native_rcl_publish)}() failed.");
            }
        }
    }
}
