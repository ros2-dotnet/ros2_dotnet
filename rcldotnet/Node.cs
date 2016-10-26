// Copyright 2016 Esteve Fernandez <esteve@apache.org>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using System.Reflection;

using ROS2dotnetUtils;

namespace rcldotnet
{
public class Node
{
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeRCLCreatePublisherHandleType(IntPtr node_handle, [MarshalAs(UnmanagedType.LPStr)] string lpString, IntPtr typesupport_ptr);

  private static NativeRCLCreatePublisherHandleType native_rcl_create_publisher_handle = null;

  private static readonly DllLoadUtils dllLoadUtils;

  static Node()
  {
    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    try {
      IntPtr nativelibrary = dllLoadUtils.LoadLibrary("rcldotnet_node__" + RCLdotnet.GetRMWIdentifier());

      IntPtr native_rcl_create_publisher_handle_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_rcl_create_publisher_handle");

      Node.native_rcl_create_publisher_handle = Marshal.GetDelegateForFunctionPointer<NativeRCLCreatePublisherHandleType>(native_rcl_create_publisher_handle_ptr);
    } catch (UnsatisfiedLinkError e) {
      System.Console.WriteLine("Native code library failed to load.\n" + e);
      Environment.Exit(1);
    }
  }

  private IntPtr node_handle_;

  public Node(IntPtr node_handle)
  {
    node_handle_ = node_handle;
  }

  public Publisher<T> CreatePublisher<T>(string topic) where T : IMessage
  {
    Type typeParametertype = typeof(T);
    MethodInfo m = typeParametertype.GetMethod("getTypeSupport");

    IntPtr typesupport = (IntPtr)m.Invoke(null, new object[] {});
    IntPtr publisher_handle = native_rcl_create_publisher_handle(node_handle_, topic, typesupport);
    Publisher<T> publisher = new Publisher<T>(node_handle_, publisher_handle);
    return publisher;
  }

  public Subscription<T> CreateSubscription<T>(string topic)
  {
    Type typeParametertype = typeof(T);
    Subscription<T> subscription = new Subscription<T>(node_handle_);
    return subscription;
  }
}
}
