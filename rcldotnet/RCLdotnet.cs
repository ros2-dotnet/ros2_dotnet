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
using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  internal static class RCLdotnetDelegates {
    internal static readonly DllLoadUtils dllLoadUtils;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLInitType ();

    internal static NativeRCLInitType native_rcl_init = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate bool NativeRCLOkType ();

    internal static NativeRCLOkType native_rcl_ok = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    internal delegate RCLRet NativeRCLCreateNodeHandleType (
      ref SafeNodeHandle nodeHandle, [MarshalAs (UnmanagedType.LPStr)] string nodeName, [MarshalAs (UnmanagedType.LPStr)] string nodeNamespace);

    internal static NativeRCLCreateNodeHandleType native_rcl_create_node_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    internal delegate RCLRet NativeRCLDestroyNodeHandleType (
      IntPtr nodeHandle);

    internal static NativeRCLDestroyNodeHandleType native_rcl_destroy_node_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLGetRMWIdentifierType ();

    internal static NativeRCLGetRMWIdentifierType native_rcl_get_rmw_identifier = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLGetZeroInitializedWaitSetType ();

    internal static NativeRCLGetZeroInitializedWaitSetType native_rcl_get_zero_initialized_wait_set = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLWaitSetInitType (
      IntPtr waitSetHandle, long numberOfSubscriptions,
      long numberOfGuardConditions, long numberOfTimers,
      long numberOfClients, long numberOfServices, long numberOfEvents);

    internal static NativeRCLWaitSetInitType native_rcl_wait_set_init = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetClearType (IntPtr waitSetHandle);

    internal static NativeRCLWaitSetClearType native_rcl_wait_set_clear = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLWaitSetAddSubscriptionType (IntPtr waitSetHandle, SafeSubscriptionHandle subscriptionHandle);

    internal static NativeRCLWaitSetAddSubscriptionType native_rcl_wait_set_add_subscription = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLWaitSetAddServiceType (IntPtr waitSetHandle, SafeServiceHandle serviceHandle);

    internal static NativeRCLWaitSetAddServiceType native_rcl_wait_set_add_service = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLWaitSetAddClientType (IntPtr waitSetHandle, SafeClientHandle clientHandle);

    internal static NativeRCLWaitSetAddClientType native_rcl_wait_set_add_client = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLDestroyWaitSetType (IntPtr waitSetHandle);

    internal static NativeRCLDestroyWaitSetType native_rcl_destroy_wait_set = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLWaitType (IntPtr waitSetHandle, long timeout);

    internal static NativeRCLWaitType native_rcl_wait = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLTakeType (SafeSubscriptionHandle subscriptionHandle, IntPtr messageHandle);

    internal static NativeRCLTakeType native_rcl_take = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate IntPtr NativeRCLCreateRequestHeaderHandleType ();

    internal static NativeRCLCreateRequestHeaderHandleType native_rcl_create_request_header_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLDestroyRequestHeaderHandleType (IntPtr requestHeaderHandle);

    internal static NativeRCLDestroyRequestHeaderHandleType native_rcl_destroy_request_header_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate long NativeRCLRequestHeaderGetSequenceNumberType (IntPtr requestHeaderHandle);

    internal static NativeRCLRequestHeaderGetSequenceNumberType native_rcl_request_header_get_sequence_number = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLTakeRequestType (SafeServiceHandle serviceHandle, IntPtr requestHeaderHandle, IntPtr requestHandle);

    internal static NativeRCLTakeRequestType native_rcl_take_request = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLSendResponseType (SafeServiceHandle serviceHandle, IntPtr requestHeaderHandle, IntPtr responseHandle);

    internal static NativeRCLSendResponseType native_rcl_send_response = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate RCLRet NativeRCLTakeResponseType (SafeClientHandle clientHandle, IntPtr requestHeaderHandle, IntPtr responseHandle);

    internal static NativeRCLTakeResponseType native_rcl_take_response = null;

    static RCLdotnetDelegates () {
      dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils ();
      string library_name = "rcldotnet";
      IntPtr pDll = dllLoadUtils.LoadLibrary (library_name);

      IntPtr native_rcl_init_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_init");
      RCLdotnetDelegates.native_rcl_init =
        (NativeRCLInitType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_init_ptr, typeof (NativeRCLInitType));

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

      IntPtr native_rcl_destroy_node_handle_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_destroy_node_handle");
      RCLdotnetDelegates.native_rcl_destroy_node_handle =
        (NativeRCLDestroyNodeHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_destroy_node_handle_ptr, typeof (NativeRCLDestroyNodeHandleType));

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
        (NativeRCLWaitSetAddSubscriptionType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_subscription_ptr, typeof (NativeRCLWaitSetAddSubscriptionType));

      IntPtr native_rcl_wait_set_add_service_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_add_service");
      RCLdotnetDelegates.native_rcl_wait_set_add_service =
        (NativeRCLWaitSetAddServiceType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_service_ptr, typeof (NativeRCLWaitSetAddServiceType));

        IntPtr native_rcl_wait_set_add_client_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_add_client");
      RCLdotnetDelegates.native_rcl_wait_set_add_client =
        (NativeRCLWaitSetAddClientType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_client_ptr, typeof (NativeRCLWaitSetAddClientType));

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

      IntPtr native_rcl_create_request_header_handle_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_create_request_header_handle");
      RCLdotnetDelegates.native_rcl_create_request_header_handle =
        (NativeRCLCreateRequestHeaderHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_request_header_handle_ptr, typeof (NativeRCLCreateRequestHeaderHandleType));

      IntPtr native_rcl_destroy_request_header_handle_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_destroy_request_header_handle");
      RCLdotnetDelegates.native_rcl_destroy_request_header_handle =
        (NativeRCLDestroyRequestHeaderHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_destroy_request_header_handle_ptr, typeof (NativeRCLDestroyRequestHeaderHandleType));

      IntPtr native_rcl_request_header_get_sequence_number_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_request_header_get_sequence_number");
      RCLdotnetDelegates.native_rcl_request_header_get_sequence_number =
        (NativeRCLRequestHeaderGetSequenceNumberType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_request_header_get_sequence_number_ptr, typeof (NativeRCLRequestHeaderGetSequenceNumberType));

      IntPtr native_rcl_take_request_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_take_request");
      RCLdotnetDelegates.native_rcl_take_request =
        (NativeRCLTakeRequestType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_take_request_ptr, typeof (NativeRCLTakeRequestType));

      IntPtr native_rcl_send_response_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_send_response");
      RCLdotnetDelegates.native_rcl_send_response =
        (NativeRCLSendResponseType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_send_response_ptr, typeof (NativeRCLSendResponseType));

      IntPtr native_rcl_take_response_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_take_response");
      RCLdotnetDelegates.native_rcl_take_response =
        (NativeRCLTakeResponseType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_take_response_ptr, typeof (NativeRCLTakeResponseType));
    }
  }

  public static class RCLdotnet {
    private static bool initialized = false;
    private static readonly object syncLock = new object ();

    public static bool Ok () {
      return RCLdotnetDelegates.native_rcl_ok ();
    }

    public static Node CreateNode (string nodeName) {
      return CreateNode (nodeName, "");
    }

    public static Node CreateNode (string nodeName, string nodeNamespace) {
      var nodeHandle = new SafeNodeHandle();
      RCLRet ret = RCLdotnetDelegates.native_rcl_create_node_handle (ref nodeHandle, nodeName, nodeNamespace);
      if (ret != RCLRet.Ok)
      {
        nodeHandle.Dispose();
        RCLExceptionHelper.ThrowFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_node_handle)}() failed.");
      }
      
      Node node = new Node (nodeHandle);
      return node;
    }

    public static void Spin (Node node) {
      while (Ok ()) {
        SpinOnce (node, 500);
      }
    }

    private static IntPtr GetZeroInitializedWaitSet () {
      return RCLdotnetDelegates.native_rcl_get_zero_initialized_wait_set ();
    }

    private static void WaitSetInit (
      IntPtr waitSetHandle,
      long numberOfSubscriptions,
      long numberOfGuardConditions,
      long numberOfTimers,
      long numberOfClients,
      long numberOfServices,
      long numberOfEvents) {
      RCLdotnetDelegates.native_rcl_wait_set_init (
        waitSetHandle, numberOfSubscriptions, numberOfGuardConditions,
        numberOfTimers, numberOfClients, numberOfServices, numberOfEvents);
    }

    private static void WaitSetClear (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_clear (waitSetHandle);
    }

    private static void WaitSetAddSubscription (IntPtr waitSetHandle, SafeSubscriptionHandle subscriptionHandle) {
      // TODO: (sh) Handle return value
      RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_subscription (waitSetHandle, subscriptionHandle);
    }

    private static void WaitSetAddService (IntPtr waitSetHandle, SafeServiceHandle serviceHandle) {
      // TODO: (sh) Handle return value
      RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_service(waitSetHandle, serviceHandle);
    }

    private static void WaitSetAddClient (IntPtr waitSetHandle, SafeClientHandle clientHandle) {
      // TODO: (sh) Handle return value
      RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_client(waitSetHandle, clientHandle);
    }

    private static void DestroyWaitSet (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_destroy_wait_set (waitSetHandle);
    }

    private static void Wait (IntPtr waitSetHandle, long timeout) {
      long ns_timeout = timeout * 1000000;
      RCLRet ret = RCLdotnetDelegates.native_rcl_wait (waitSetHandle, ns_timeout);
      // TODO(esteve): do something with ret
    }

    private static bool Take (SafeSubscriptionHandle subscriptionHandle, IMessage message) {
      bool status = false;
      IntPtr messageHandle = message._CREATE_NATIVE_MESSAGE ();
      RCLRet ret = RCLdotnetDelegates.native_rcl_take (subscriptionHandle, messageHandle);
      
      // TODO: (sh) Handle all return values (exceptions)
      switch (ret) {
        case RCLRet.Ok:
          message._READ_HANDLE (messageHandle);
          status = true;
          break;
        case RCLRet.SubscriptionTakeFailed:
          status = false;
          break;
        default:
          break;
      }
      message._DESTROY_NATIVE_MESSAGE (messageHandle);
      return status;
    }

    private static bool TakeRequest(SafeServiceHandle serviceHandle, IntPtr requestHeaderHandle, IMessage request) {
      bool status = false;
      IntPtr requestHandle = request._CREATE_NATIVE_MESSAGE();
      RCLRet ret = RCLdotnetDelegates.native_rcl_take_request(serviceHandle, requestHeaderHandle, requestHandle);
      
      // TODO: (sh) Handle all return values (exceptions)
      switch (ret)
      {
        case RCLRet.Ok:
          request._READ_HANDLE(requestHandle);
          status = true;
          break;
        case RCLRet.ServiceTakeFailed:
          status = false;
          break;
        default:
          break;
      }

      // TODO: (sh) don't leak memory on exceptions
      request._DESTROY_NATIVE_MESSAGE (requestHandle);
      return status;
    }

    private static bool TakeResponse(SafeClientHandle clientHandle, IntPtr requestHeaderHandle, IMessage response) {
      bool status = false;
      IntPtr responseHandle = response._CREATE_NATIVE_MESSAGE();
      RCLRet ret = RCLdotnetDelegates.native_rcl_take_response(clientHandle, requestHeaderHandle, responseHandle);
      
      // TODO: (sh) Handle all return values (exceptions)
      switch (ret)
      {
        case RCLRet.Ok:
          response._READ_HANDLE(responseHandle);
          status = true;
          break;
        case RCLRet.ClientTakeFailed:
          status = false;
          break;
        default:
          break;
      }

      // TODO: (sh) don't leak memory on exceptions
      response._DESTROY_NATIVE_MESSAGE (responseHandle);
      return status;
    }

    private static void SendResponse(SafeServiceHandle serviceHandle, IntPtr requestHeaderHandle, IMessage response)
    {
      IntPtr responseHandle = response._CREATE_NATIVE_MESSAGE();
      response._WRITE_HANDLE (responseHandle);

      // TODO: (sh) check return values
      RCLRet ret = RCLdotnetDelegates.native_rcl_send_response(serviceHandle, requestHeaderHandle, responseHandle);

      // TODO: (sh) don't leak memory on exceptions
      response._DESTROY_NATIVE_MESSAGE (responseHandle);
    }

    public static void SpinOnce (Node node, long timeout) {
      IntPtr waitSetHandle = GetZeroInitializedWaitSet ();

      long numberOfSubscriptions = node.Subscriptions.Count;
      long numberOfGuardConditions = 0;
      long numberOfTimers = 0;
      long numberOfClients = node.Clients.Count;
      long numberOfServices = node.Services.Count;
      long numberOfEvents = 0;

      WaitSetInit (
        waitSetHandle,
        numberOfSubscriptions,
        numberOfGuardConditions,
        numberOfTimers,
        numberOfClients,
        numberOfServices,
        numberOfEvents
      );

      WaitSetClear (waitSetHandle);

      foreach (Subscription subscription in node.Subscriptions) {
        WaitSetAddSubscription (waitSetHandle, subscription.Handle);
      }

      foreach (var service in node.Services)
      {
        WaitSetAddService(waitSetHandle, service.Handle);
      }

      foreach (var client in node.Clients)
      {
        WaitSetAddClient(waitSetHandle, client.Handle);
      }

      Wait (waitSetHandle, timeout);

      foreach (Subscription subscription in node.Subscriptions) {
        IMessage message = subscription.CreateMessage ();
        bool result = Take (subscription.Handle, message);
        if (result) {
          subscription.TriggerCallback (message);
        }
      }
      
      // gets reused for each element in the for loop.
      IntPtr requestHeaderHandle = RCLdotnetDelegates.native_rcl_create_request_header_handle();

      foreach (var service in node.Services)
      {
        var request = service.CreateRequest();
        var response = service.CreateResponse();

        var result = TakeRequest(service.Handle, requestHeaderHandle, request);
        if (result)
        {
          // TODO: (sh) catch exceptions
          service.TriggerCallback(request, response);

          SendResponse(service.Handle, requestHeaderHandle, response);
        }
      }

      foreach (var client in node.Clients)
      {
        var response = client.CreateResponse();

        var result = TakeResponse(client.Handle, requestHeaderHandle, response);
        if (result)
        {
          var sequenceNumber = RCLdotnetDelegates.native_rcl_request_header_get_sequence_number(requestHeaderHandle);
          client.HandleResponse(sequenceNumber, response);
        }
      }

      // TODO: (sh) don't leak memory on exceptions
      RCLdotnetDelegates.native_rcl_destroy_request_header_handle(requestHeaderHandle);

      DestroyWaitSet (waitSetHandle);
    }

    public static void Init () {
      lock (syncLock) {
        if (!initialized) {
          // TODO: (sh) Handle return value
          RCLRet ret = RCLdotnetDelegates.native_rcl_init ();
          initialized = true;
        }
      }
    }

    public static string GetRMWIdentifier () {
      IntPtr ptr = RCLdotnetDelegates.native_rcl_get_rmw_identifier ();
      string rmw_identifier = Marshal.PtrToStringAnsi (ptr);
      return rmw_identifier;
    }
  }
}
