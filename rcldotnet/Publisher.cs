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
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  internal class PublisherDelegates {
    internal static readonly DllLoadUtils dllLoadUtils;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLPublishType (
      IntPtr publisher_handle, IntPtr message_struct);

    internal static NativeRCLPublishType native_rcl_publish = null;

    static PublisherDelegates () {
      dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils ();
      try {
        IntPtr nativelibrary = dllLoadUtils.LoadLibrary ("rcldotnet_publisher");
        IntPtr native_rcl_publish_ptr = dllLoadUtils.GetProcAddress (nativelibrary, "native_rcl_publish");
        PublisherDelegates.native_rcl_publish = (NativeRCLPublishType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_publish_ptr, typeof (NativeRCLPublishType));
      } catch (UnsatisfiedLinkError e) {
        System.Console.WriteLine ("Native code library failed to load.\n" + e);
        Environment.Exit (1);
      }
    }
  }

  public class Publisher<T> : IPublisher<T>
  where T : IMessage {

    private IntPtr publisher_handle_;

    public Publisher (IntPtr publisher_handle) {
      publisher_handle_ = publisher_handle;
    }

    public IntPtr Handle { get { return publisher_handle_; } }

    public void Publish (T msg) {
      IntPtr message_handle = msg._CREATE_NATIVE_MESSAGE ();

      msg._WRITE_HANDLE (message_handle);

      PublisherDelegates.native_rcl_publish (publisher_handle_, message_handle);

      msg._DESTROY_NATIVE_MESSAGE (message_handle);
    }
  }
}
