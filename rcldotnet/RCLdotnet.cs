// Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
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
    internal delegate void NativeRCLInitType ();

    internal static NativeRCLInitType native_rcl_init = null;

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
    internal delegate IntPtr NativeRCLGetZeroInitializedWaitSetType ();

    internal static NativeRCLGetZeroInitializedWaitSetType native_rcl_get_zero_initialized_wait_set = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLWaitSetInitType (
      IntPtr waitSetHandle, long numberOfSubscriptions,
      long numberOfGuardConditions, long numberOfTimers,
      long numberOfClients, long numberOfServices);

    internal static NativeRCLWaitSetInitType native_rcl_wait_set_init = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetClearSubscriptionsType (IntPtr waitSetHandle);

    internal static NativeRCLWaitSetClearSubscriptionsType native_rcl_wait_set_clear_subscriptions = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetClearServicesType (IntPtr waitSetHandle);

    internal static NativeRCLWaitSetClearServicesType native_rcl_wait_set_clear_services = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetClearClientsType (IntPtr waitSetHandle);

    internal static NativeRCLWaitSetClearClientsType native_rcl_wait_set_clear_clients = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLWaitSetAddSubscriptionType (IntPtr waitSetHandle, IntPtr subscriptionHandle);

    internal static NativeRCLWaitSetAddSubscriptionType native_rcl_wait_set_add_subscription = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate void NativeRCLDestroyWaitSetType (IntPtr waitSetHandle);

    internal static NativeRCLDestroyWaitSetType native_rcl_destroy_wait_set = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLWaitType (IntPtr waitSetHandle, long timeout);

    internal static NativeRCLWaitType native_rcl_wait = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLTakeType (IntPtr subscriptionHandle, IntPtr messageHandle);

    internal static NativeRCLTakeType native_rcl_take = null;

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

      IntPtr native_rcl_wait_set_clear_subscriptions_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_clear_subscriptions");
      RCLdotnetDelegates.native_rcl_wait_set_clear_subscriptions =
        (NativeRCLWaitSetClearSubscriptionsType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_clear_subscriptions_ptr, typeof (NativeRCLWaitSetClearSubscriptionsType));

      IntPtr native_rcl_wait_set_clear_services_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_clear_services");
      RCLdotnetDelegates.native_rcl_wait_set_clear_services =
        (NativeRCLWaitSetClearServicesType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_clear_services_ptr, typeof (NativeRCLWaitSetClearServicesType));

      IntPtr native_rcl_wait_set_clear_clients_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_clear_clients");
      RCLdotnetDelegates.native_rcl_wait_set_clear_clients =
        (NativeRCLWaitSetClearClientsType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_clear_clients_ptr, typeof (NativeRCLWaitSetClearClientsType));

      IntPtr native_rcl_wait_set_add_subscription_ptr =
        dllLoadUtils.GetProcAddress (pDll, "native_rcl_wait_set_add_subscription");
      RCLdotnetDelegates.native_rcl_wait_set_add_subscription =
        (NativeRCLWaitSetAddSubscriptionType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_wait_set_add_subscription_ptr, typeof (NativeRCLWaitSetAddSubscriptionType));

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
      int ret = RCLdotnetDelegates.native_rcl_create_node_handle (ref nodeHandle, nodeName, nodeNamespace);
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
      long numberOfServices) {
      RCLdotnetDelegates.native_rcl_wait_set_init (
        waitSetHandle, numberOfSubscriptions, numberOfGuardConditions,
        numberOfTimers, numberOfClients, numberOfServices);
    }

    private static void WaitSetClearSubscriptions (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_clear_subscriptions (waitSetHandle);
    }

    private static void WaitSetClearServices (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_clear_services (waitSetHandle);
    }

    private static void WaitSetClearClients (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_clear_clients (waitSetHandle);
    }

    private static void WaitSetAddSubscription (IntPtr waitSetHandle, IntPtr subscriptionHandle) {
      RCLdotnetDelegates.native_rcl_wait_set_add_subscription (waitSetHandle, subscriptionHandle);
    }

    private static void DestroyWaitSet (IntPtr waitSetHandle) {
      RCLdotnetDelegates.native_rcl_destroy_wait_set (waitSetHandle);
    }

    private static void Wait (IntPtr waitSetHandle, long timeout) {
      long ns_timeout = timeout * 1000000;
      RCLRet ret = (RCLRet) RCLdotnetDelegates.native_rcl_wait (waitSetHandle, ns_timeout);
      // TODO(esteve): do something with ret
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
          break;
        default:
          break;
      }
      message._DESTROY_NATIVE_MESSAGE (messageHandle);
      return status;
    }

    public static void SpinOnce (Node node, long timeout) {
      IntPtr waitSetHandle = GetZeroInitializedWaitSet ();

      long numberOfSubscriptions = node.Subscriptions.Count;
      long numberOfGuardConditions = 0;
      long numberOfTimers = 0;
      long numberOfClients = 0;
      long numberOfServices = 0;

      WaitSetInit (
        waitSetHandle,
        numberOfSubscriptions,
        numberOfGuardConditions,
        numberOfTimers,
        numberOfClients,
        numberOfServices
      );

      WaitSetClearSubscriptions (waitSetHandle);

      WaitSetClearServices (waitSetHandle);

      WaitSetClearClients (waitSetHandle);

      foreach (ISubscription subscription in node.Subscriptions) {
        WaitSetAddSubscription (waitSetHandle, subscription.Handle);
      }

      Wait (waitSetHandle, timeout);

      foreach (ISubscription subscription in node.Subscriptions) {
        IMessage message = subscription.CreateMessage ();
        bool result = Take (subscription.Handle, message);
        if (result) {
          subscription.CallbackFN (message);
        }
      }
      DestroyWaitSet (waitSetHandle);
    }

    public static void Init () {
      lock (syncLock) {
        if (!initialized) {
          RCLdotnetDelegates.native_rcl_init ();
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