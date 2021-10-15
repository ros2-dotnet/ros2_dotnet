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
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  internal class RCLdotnetDelegates {
    internal static readonly DllLoadUtils dllLoadUtils;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLInitType ();

    internal static NativeRCLInitType native_rcl_init = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLShutdownType ();

    internal static NativeRCLShutdownType native_rcl_shutdown = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate bool NativeRCLOkType ();

    internal static NativeRCLOkType native_rcl_ok = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    internal delegate int NativeRCLCreateNodeHandleType (
      ref IntPtr nodeHandle, [MarshalAs (UnmanagedType.LPStr)] string nodeName, [MarshalAs (UnmanagedType.LPStr)] string nodeNamespace);

    internal static NativeRCLCreateNodeHandleType native_rcl_create_node_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLGetRMWIdentifierType ();

    internal static NativeRCLGetRMWIdentifierType native_rcl_get_rmw_identifier = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLGetErrorStringType (int len, StringBuilder lpBuffer);

    internal static NativeRCLGetErrorStringType native_rcl_get_error_string = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLResetErrorType ();

    internal static NativeRCLResetErrorType native_rcl_reset_error = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLGetZeroInitializedWaitSetType ();

    internal static NativeRCLGetZeroInitializedWaitSetType native_rcl_get_zero_initialized_wait_set = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLWaitSetInitType (
      IntPtr waitSetHandle, long numberOfSubscriptions,
      long numberOfGuardConditions, long numberOfTimers,
      long numberOfClients, long numberOfServices, long numberOfEvents);

    internal static NativeRCLWaitSetInitType native_rcl_wait_set_init = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetClearType (IntPtr waitSetHandle);

    internal static NativeRCLWaitSetClearType native_rcl_wait_set_clear = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetAddType (IntPtr waitSetHandle, IntPtr handle);

    internal static NativeRCLWaitSetAddType native_rcl_wait_set_add_subscription = null;

    internal static NativeRCLWaitSetAddType native_rcl_wait_set_add_service = null;

    internal static NativeRCLWaitSetAddType native_rcl_wait_set_add_client = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLDestroyWaitSetType (IntPtr waitSetHandle);

    internal static NativeRCLDestroyWaitSetType native_rcl_destroy_wait_set = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLWaitType (IntPtr waitSetHandle, long timeout);

    internal static NativeRCLWaitType native_rcl_wait = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLTakeType (IntPtr subscriptionHandle, IntPtr messageHandle);

    internal static NativeRCLTakeType native_rcl_take = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLServiceType (IntPtr serviceHandle, IntPtr requestHeader, IntPtr messageHandle);

    internal static NativeRCLServiceType native_rcl_take_request = null;

    internal static NativeRCLServiceType native_rcl_send_response = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLCreateRequestHeaderType (ref IntPtr header);

    internal static NativeRCLCreateRequestHeaderType native_rcl_create_request_header = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLFreeRequestHeaderType (IntPtr header);

    internal static NativeRCLFreeRequestHeaderType native_rcl_free_request_header = null;

    static RCLdotnetDelegates () {
      dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils ();
      string library_name = "rcldotnet";
      IntPtr pDll = dllLoadUtils.LoadLibrary (library_name);

      IntPtr native_rcl_init_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_init");
      RCLdotnetDelegates.native_rcl_init =
        (NativeRCLInitType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_init_ptr, typeof (NativeRCLInitType));

      IntPtr native_rcl_shutdown_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_shutdown");
      RCLdotnetDelegates.native_rcl_shutdown =
        (NativeRCLShutdownType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_shutdown_ptr, typeof (NativeRCLShutdownType));

      IntPtr native_rcl_get_error_string_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_get_error_string");
      RCLdotnetDelegates.native_rcl_get_error_string =
        (NativeRCLGetErrorStringType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_get_error_string_ptr, typeof (NativeRCLGetErrorStringType));

      IntPtr native_rcl_reset_error_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_reset_error");
      RCLdotnetDelegates.native_rcl_reset_error =
        (NativeRCLResetErrorType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_reset_error_ptr, typeof (NativeRCLResetErrorType));

      IntPtr native_rcl_get_rmw_identifier_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_get_rmw_identifier");
      RCLdotnetDelegates.native_rcl_get_rmw_identifier =
        (NativeRCLGetRMWIdentifierType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_get_rmw_identifier_ptr, typeof (NativeRCLGetRMWIdentifierType));

      IntPtr native_rcl_ok_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_ok");
      RCLdotnetDelegates.native_rcl_ok =
        (NativeRCLOkType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_ok_ptr, typeof (NativeRCLOkType));

      IntPtr native_rcl_create_node_handle_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_create_node_handle");
      RCLdotnetDelegates.native_rcl_create_node_handle =
        (NativeRCLCreateNodeHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_node_handle_ptr, typeof (NativeRCLCreateNodeHandleType));

      IntPtr native_rcl_get_zero_initialized_wait_set_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_get_zero_initialized_wait_set");
      RCLdotnetDelegates.native_rcl_get_zero_initialized_wait_set =
        (NativeRCLGetZeroInitializedWaitSetType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_get_zero_initialized_wait_set_ptr, typeof (NativeRCLGetZeroInitializedWaitSetType));

      IntPtr native_rcl_wait_set_init_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_init");
      RCLdotnetDelegates.native_rcl_wait_set_init =
        (NativeRCLWaitSetInitType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_init_ptr, typeof (NativeRCLWaitSetInitType));

      IntPtr native_rcl_wait_set_clear_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_clear");
      RCLdotnetDelegates.native_rcl_wait_set_clear =
        (NativeRCLWaitSetClearType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_clear_ptr, typeof (NativeRCLWaitSetClearType));

      IntPtr native_rcl_wait_set_add_subscription_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_add_subscription");
      RCLdotnetDelegates.native_rcl_wait_set_add_subscription =
        (NativeRCLWaitSetAddType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_subscription_ptr, typeof (NativeRCLWaitSetAddType));

      IntPtr native_rcl_wait_set_add_service_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_add_service");
      RCLdotnetDelegates.native_rcl_wait_set_add_service =
        (NativeRCLWaitSetAddType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_service_ptr, typeof (NativeRCLWaitSetAddType));

      IntPtr native_rcl_wait_set_add_client_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_add_client");
      RCLdotnetDelegates.native_rcl_wait_set_add_client =
        (NativeRCLWaitSetAddType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_client_ptr, typeof (NativeRCLWaitSetAddType));

      IntPtr native_rcl_destroy_wait_set_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_destroy_wait_set");
      RCLdotnetDelegates.native_rcl_destroy_wait_set =
        (NativeRCLDestroyWaitSetType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_destroy_wait_set_ptr, typeof (NativeRCLDestroyWaitSetType));

      IntPtr native_rcl_wait_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait");
      RCLdotnetDelegates.native_rcl_wait =
        (NativeRCLWaitType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_ptr, typeof (NativeRCLWaitType)
        );

      IntPtr native_rcl_take_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_take");
      RCLdotnetDelegates.native_rcl_take =
        (NativeRCLTakeType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_take_ptr, typeof (NativeRCLTakeType));
    
      IntPtr native_rcl_take_request_ptr = 
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_take_request");
      RCLdotnetDelegates.native_rcl_take_request = 
        (NativeRCLServiceType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_take_request_ptr, typeof (NativeRCLServiceType));

      IntPtr native_rcl_send_response_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_send_response");
      RCLdotnetDelegates.native_rcl_send_response = 
        (NativeRCLServiceType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_send_response_ptr, typeof (NativeRCLServiceType));
      
      IntPtr native_rcl_create_request_header_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_create_request_header");
      RCLdotnetDelegates.native_rcl_create_request_header = 
        (NativeRCLCreateRequestHeaderType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_request_header_ptr, typeof (NativeRCLCreateRequestHeaderType));
      
      IntPtr native_rcl_free_request_header_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_free_request_header");
      RCLdotnetDelegates.native_rcl_free_request_header = 
        (NativeRCLFreeRequestHeaderType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_free_request_header_ptr, typeof (NativeRCLFreeRequestHeaderType));
    }
  }

  public class RCLdotnet {
    private static bool initialized = false;
    private static readonly object syncLock = new object ();

    public static bool Ok () {
      return RCLdotnetDelegates.native_rcl_ok ();
    }

    public static Node CreateNode (string nodeName) {
      return CreateNode (nodeName, "");
    }

    public static Node CreateNode (string nodeName, string nodeNamespace) {
      IntPtr nodeHandle = IntPtr.Zero;
      RCLRet ret = (RCLRet) RCLdotnetDelegates.native_rcl_create_node_handle (ref nodeHandle, nodeName, nodeNamespace);
      if (ret != RCLRet.Ok)
      {
          // TODO: SHS: Log this somewhere.
          ResetRclError();
          return null;
      }

      Node node = new Node (nodeHandle);
      return node;
    }

    public static IntPtr CreateRequestHeader ()
    {
      IntPtr header = IntPtr.Zero;
      RCLdotnetDelegates.native_rcl_create_request_header (ref header);
      return header;
    }

    public static void FreeRequestHeader (IntPtr header)
    {
      RCLdotnetDelegates.native_rcl_free_request_header (header);      
    }

    public static void Spin (INode node) {
      while (Ok ()) {
        SpinOnce (node, 500);
      }
    }

    public static IntPtr GetZeroInitializedWaitSet () {
      return RCLdotnetDelegates.native_rcl_get_zero_initialized_wait_set ();
    }

    public static bool WaitSetInit (
      IntPtr waitSetHandle,
      long numberOfSubscriptions,
      long numberOfGuardConditions,
      long numberOfTimers,
      long numberOfClients,
      long numberOfServices,
      long numberOfEvents) {
      RCLRet ret = (RCLRet) RCLdotnetDelegates.native_rcl_wait_set_init (
        waitSetHandle, numberOfSubscriptions, numberOfGuardConditions,
        numberOfTimers, numberOfClients, numberOfServices, numberOfEvents);
      if (ret != RCLRet.Ok)
      {
          // TODO: SHS: Log this somewhere.
          ResetRclError();
          return false;
      }

      return true;
    }

    public static void WaitSetClear (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_clear (waitSetHandle);
    }

    public static void WaitSetAddSubscription (IntPtr waitSetHandle, IntPtr subscriptionHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_add_subscription (waitSetHandle, subscriptionHandle);
    }

    public static void WaitSetAddService (IntPtr waitSetHandle, IntPtr serviceHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_add_service (waitSetHandle, serviceHandle);
    }

    public static void WaitSetAddClient (IntPtr waitSetHandle, IntPtr clientHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_add_client (waitSetHandle, clientHandle);
    }

    public static void DestroyWaitSet (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_destroy_wait_set (waitSetHandle);
    }

    public static int Wait (IntPtr waitSetHandle, long timeout) {
      long ns_timeout = timeout * 1000000;
      return RCLdotnetDelegates.native_rcl_wait (waitSetHandle, ns_timeout);
    }

    private static bool Take (IntPtr subscriptionHandle, IMessage message) {
      bool status = false;
      IntPtr messageHandle = message._CREATE_NATIVE_MESSAGE ();
      RCLRet ret = (RCLRet) RCLdotnetDelegates.native_rcl_take (subscriptionHandle, messageHandle);
      switch (ret) {
        case RCLRet.Ok:
          message._READ_HANDLE (messageHandle);
          status = true;
          break;
        case RCLRet.SubscriptionTakeFailed:
          status = false;
          // TODO: SHS: Log this somewhere.
          ResetRclError();
          break;
        default:
          break;
      }
      message._DESTROY_NATIVE_MESSAGE (messageHandle);
      return status;
    }

    private static bool TakeRequest (IntPtr serviceHandle, IntPtr requestHeader, IMessage message) {
      bool status = false;
      IntPtr messageHandle = message._CREATE_NATIVE_MESSAGE ();
      RCLRet ret = (RCLRet) RCLdotnetDelegates.native_rcl_take_request (serviceHandle, requestHeader, messageHandle);
      switch (ret) {
        case RCLRet.Ok:
          message._READ_HANDLE (messageHandle);
          status = true;
          break;
        case RCLRet.ServiceTakeFailed:
          status = false;
          // TODO: SHS: Log this somewhere.
          ResetRclError();
          break;
        default:
          break;
      }
      message._DESTROY_NATIVE_MESSAGE (messageHandle);
      return status;
    }

    private static bool SendResponse (IntPtr serviceHandle, IntPtr requestHeader, IMessage message) {
      bool status = false;
      IntPtr messageHandle = message._CREATE_NATIVE_MESSAGE ();
      message._WRITE_HANDLE (messageHandle);
      RCLRet ret = (RCLRet) RCLdotnetDelegates.native_rcl_send_response (serviceHandle, requestHeader, messageHandle);
      switch (ret) {
        case RCLRet.Ok:          
          status = true;
          break;
        case RCLRet.Error:
          status = false;
          // TODO: SHS: Log this somewhere.
          ResetRclError();
          break;
        default:
          break;
      }
      message._DESTROY_NATIVE_MESSAGE (messageHandle);
      return status;  
    }

    public static void SpinOnce (INode node, long timeout) {
      IntPtr waitSetHandle = GetZeroInitializedWaitSet ();

      // If this node does not have anything to wait on, just sleep
      if (node.Subscriptions.Count + node.Services.Count == 0)
      {
        System.Threading.Thread.Sleep((int)timeout);
        return;
      }

      long numberOfSubscriptions = node.Subscriptions.Count;
      long numberOfGuardConditions = 0;
      long numberOfTimers = 0;
      long numberOfClients = 0;
      long numberOfServices = node.Services.Count;
      long numberOfEvents = 0;

      if (!WaitSetInit (
        waitSetHandle,
        numberOfSubscriptions,
        numberOfGuardConditions,
        numberOfTimers,
        numberOfClients,
        numberOfServices,
        numberOfEvents
      )) {
        // We will spin again
        return;
      }

      WaitSetClear (waitSetHandle);

      foreach (ISubscriptionBase subscription in node.Subscriptions) {
        WaitSetAddSubscription (waitSetHandle, subscription.Handle);
      }

      foreach (IServiceBase service in node.Services) {
        WaitSetAddService (waitSetHandle, service.Handle);
      }

      RCLRet ret = (RCLRet) Wait (waitSetHandle, timeout);
      if (ret != RCLRet.Ok)
      {
          // TODO: SHS: Log this somewhere.
          ResetRclError();
          DestroyWaitSet (waitSetHandle);
          return;
      }

      foreach (ISubscriptionBase subscription in node.Subscriptions) {
        IMessage message = subscription.CreateMessage ();
        bool result = Take (subscription.Handle, message);
        if (result) {
          subscription.TriggerCallback (message);
        }
      }

      foreach (IServiceBase service in node.Services) {
        IMessage request = service.CreateRequestMessage ();
        IntPtr header = CreateRequestHeader();
        bool result = TakeRequest (service.Handle, header, request);
        if (result) {
          var response = service.HandleRequest (request);
          SendResponse (service.Handle, header, response);
        }
        FreeRequestHeader(header);
      }

      DestroyWaitSet (waitSetHandle);
    }

    public static RCLRet Init () {
      RCLRet ret = RCLRet.Ok;
      lock (syncLock) {
        if (!initialized) {
          ret = (RCLRet)RCLdotnetDelegates.native_rcl_init ();
          if (ret == RCLRet.Ok)
          {
            initialized = true;
          }
        }
      }

      return ret;
    }

    public static RCLRet Shutdown()
    {
      RCLRet ret = RCLRet.Ok;
      lock (syncLock)
      {
        if (initialized)
        {
          ret = (RCLRet)RCLdotnetDelegates.native_rcl_shutdown();
          if (ret == RCLRet.Ok)
          {
            initialized = false;
          }
        }
      }

      return ret;
    }

    public static string GetErrorString () {
      StringBuilder buffer = new StringBuilder(1024);
      RCLdotnetDelegates.native_rcl_get_error_string (buffer.Capacity, buffer);
      return buffer.ToString();
    }

    public static void ResetRclError () {
      RCLdotnetDelegates.native_rcl_reset_error ();
    }

    public static string GetRMWIdentifier () {
      IntPtr ptr = RCLdotnetDelegates.native_rcl_get_rmw_identifier ();
      string rmw_identifier = Marshal.PtrToStringAnsi (ptr);
      return rmw_identifier;
    }
  }
}
