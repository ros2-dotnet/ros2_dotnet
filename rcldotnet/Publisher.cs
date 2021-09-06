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
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  internal static class PublisherDelegates {
    internal static readonly DllLoadUtils dllLoadUtils;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLPublishType (
      IntPtr publisherHandle, IntPtr messageHandle);

    internal static NativeRCLPublishType native_rcl_publish = null;

    static PublisherDelegates () {
      dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils ();
      IntPtr nativelibrary = dllLoadUtils.LoadLibrary ("rcldotnet_publisher");
      IntPtr native_rcl_publish_ptr = dllLoadUtils.GetProcAddress (nativelibrary, "native_rcl_publish");
      PublisherDelegates.native_rcl_publish = (NativeRCLPublishType) Marshal.GetDelegateForFunctionPointer (
        native_rcl_publish_ptr, typeof (NativeRCLPublishType));
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
  }

  public sealed class Publisher<T> : Publisher
    where T : IMessage {

      internal Publisher (IntPtr handle) {
        Handle = handle;
      }

      internal IntPtr Handle { get; }

      public void Publish (T msg) {
        IntPtr messageHandle = msg._CREATE_NATIVE_MESSAGE ();

        msg._WRITE_HANDLE (messageHandle);

        PublisherDelegates.native_rcl_publish (Handle, messageHandle);

        msg._DESTROY_NATIVE_MESSAGE (messageHandle);
      }
    }
}
