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

public class PublisherDelegates
{

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  public delegate void NativeRCLPublishType(IntPtr publisher_handle, IntPtr message_struct, IntPtr from_dotnet_converter);
  //public delegate void NativeRCLPublishType(IntPtr publisher_handle, IMessageStruct message_struct, IntPtr from_dotnet_converter);

  public static NativeRCLPublishType native_rcl_publish = null;

  private static readonly DllLoadUtils dllLoadUtils;

  static PublisherDelegates()
  {
    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    try {
      IntPtr nativelibrary = dllLoadUtils.LoadLibrary("rcldotnet_publisher__" + RCLdotnet.GetRMWIdentifier());
      Console.WriteLine("PTR PUB nativelibrary: " + nativelibrary);

      IntPtr native_rcl_publish_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_rcl_publish");
      Console.WriteLine("PTR PUB native_rcl_publish_ptr: " + native_rcl_publish_ptr);

      PublisherDelegates.native_rcl_publish = Marshal.GetDelegateForFunctionPointer<NativeRCLPublishType>(native_rcl_publish_ptr);
    } catch (UnsatisfiedLinkError e) {
      System.Console.WriteLine("Native code library failed to load.\n" + e);
      Environment.Exit(1);
    }
  }

}


public class Publisher<T> where T : IMessage
{

  private IntPtr node_handle_;

  private IntPtr publisher_handle_;

  public Publisher(IntPtr node_handle, IntPtr publisher_handle)
  {
    node_handle_ = node_handle;
    publisher_handle_ = publisher_handle;
  }

  public void Publish(T msg)
  {
    Type typeParametertype = typeof(T);
    MethodInfo m = typeParametertype.GetMethod("getFromDotnetConverter");
    IntPtr converter_ptr = (IntPtr)m.Invoke(null, new object[] {});
    Console.WriteLine("CONVERTER PTR: " + converter_ptr.ToInt64());
    Console.WriteLine("PUB PTR: " + publisher_handle_.ToInt64());

    int msg_struct_size = Marshal.SizeOf(msg.MessageStruct);
    IntPtr pnt = Marshal.AllocHGlobal(msg_struct_size);
    Marshal.StructureToPtr(msg.MessageStruct, pnt, false);

    Console.WriteLine("SIZE OF: " + msg_struct_size);

    //PublisherDelegates.native_rcl_publish(publisher_handle_, msg.MessageStruct, converter_ptr);
    PublisherDelegates.native_rcl_publish(publisher_handle_, pnt, converter_ptr);

    Marshal.FreeHGlobal(pnt);
  }
}
}
