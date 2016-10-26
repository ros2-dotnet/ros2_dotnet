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
using System.Collections.Concurrent;

using ROS2dotnetUtils;

namespace rcldotnet
{
public class RCLdotnet
{
  private static string rmwImplementation = null;
  private static bool initialized = false;
  private static readonly object syncLock = new object();

  private static readonly ConcurrentDictionary<string, string> rmwToTypesupport = new ConcurrentDictionary<string, string>(
  new List<KeyValuePair<string, string>>() {
    new KeyValuePair<string, string>("rmw_fastrtps_cpp", "rosidl_typesupport_introspection_c"),
        new KeyValuePair<string, string>("rmw_opensplice_cpp", "rosidl_typesupport_opensplice_c"),
        new KeyValuePair<string, string>("rmw_connext_cpp", "rosidl_typesupport_connext_c"),
        new KeyValuePair<string, string>("rmw_connext_dynamic_cpp", "rosidl_typesupport_introspection_c")
  });

  public static bool Ok()
  {
    return native_rcl_ok();
  }

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeRCLInitType();

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeRCLGetRMWIdentifierType();

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate bool NativeRCLOkType();

  [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet=CharSet.Ansi)]
  private delegate IntPtr NativeRCLCreateNodeHandleType([MarshalAs(UnmanagedType.LPStr)] string lpString);

  private static NativeRCLInitType native_rcl_init = null;

  private static NativeRCLGetRMWIdentifierType native_rcl_get_rmw_identifier = null;

  private static NativeRCLOkType native_rcl_ok = null;

  private static NativeRCLCreateNodeHandleType native_rcl_create_node_handle = null;

  public static void Init()
  {
    lock(syncLock)
    {
      if (!initialized) {
        if (RCLdotnet.rmwImplementation == null) {
          foreach(KeyValuePair<string, string> entry in rmwToTypesupport)
          {
            try {
              setRMWImplementation(entry.Key);
              break;
            } catch(UnsatisfiedLinkError) {
              // TODO(esteve): handle exception
            }
          }

          if(RCLdotnet.rmwImplementation == null)
          {
            Console.WriteLine("No RMW implementation found");
            Environment.Exit(1);
          } else {
            native_rcl_init();
            initialized = true;
          }
        }
      }
    }
  }

  public static void setRMWImplementation(string rmwImplementation) {
    lock(syncLock) {
      DllLoadUtils dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
      string library_name = "rcldotnet__" + rmwImplementation;
      IntPtr pDll = dllLoadUtils.LoadLibrary(library_name);
      IntPtr native_rcl_init_ptr = dllLoadUtils.GetProcAddress(pDll, "native_rcl_init");
      IntPtr native_rcl_get_rmw_identifier_ptr = dllLoadUtils.GetProcAddress(pDll, "native_rcl_get_rmw_identifier");
      IntPtr native_rcl_ok_ptr = dllLoadUtils.GetProcAddress(pDll, "native_rcl_ok");
      IntPtr native_rcl_create_node_handle_ptr = dllLoadUtils.GetProcAddress(pDll, "native_rcl_create_node_handle");

      RCLdotnet.native_rcl_init = Marshal.GetDelegateForFunctionPointer<NativeRCLInitType>(native_rcl_init_ptr);
      RCLdotnet.native_rcl_get_rmw_identifier = Marshal.GetDelegateForFunctionPointer<NativeRCLGetRMWIdentifierType>(native_rcl_get_rmw_identifier_ptr);
      RCLdotnet.native_rcl_ok = Marshal.GetDelegateForFunctionPointer<NativeRCLOkType>(native_rcl_ok_ptr);
      RCLdotnet.native_rcl_create_node_handle = Marshal.GetDelegateForFunctionPointer<NativeRCLCreateNodeHandleType>(native_rcl_create_node_handle_ptr);
      RCLdotnet.rmwImplementation = rmwImplementation;
    }
  }

  public static Node CreateNode(string node_name)
  {
    IntPtr node_handle = native_rcl_create_node_handle(node_name);
    Node node = new Node(node_handle);
    return node;
  }

  public static string GetRMWIdentifier()
  {
    IntPtr ptr = native_rcl_get_rmw_identifier();
    string rmw_identifier = Marshal.PtrToStringAnsi(ptr);
    return rmw_identifier;
  }

  public static string GetTypesupportIdentifier() {
    return rmwToTypesupport[GetRMWIdentifier()];
  }
}
}
