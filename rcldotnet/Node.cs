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
using System.Reflection;
using System.Runtime.InteropServices;
using ROS2.Common;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2 {
  internal static class NodeDelegates {
    private static readonly DllLoadUtils dllLoadUtils;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLCreatePublisherHandleType (
      ref IntPtr publisherHandle, IntPtr nodeHandle, [MarshalAs (UnmanagedType.LPStr)] string nodeName, IntPtr typesupportHandle);

    internal static NativeRCLCreatePublisherHandleType native_rcl_create_publisher_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLCreateSubscriptionHandleType (
      ref IntPtr subscriptionHandle, IntPtr nodeHandle, [MarshalAs (UnmanagedType.LPStr)] string nodeName, IntPtr typesupportHandle);

    internal static NativeRCLCreateSubscriptionHandleType native_rcl_create_subscription_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLCreateServiceHandleType (
      ref IntPtr serviceHandle, IntPtr nodeHandle, [MarshalAs (UnmanagedType.LPStr)] string serviceName, IntPtr typesupportHandle);

    internal static NativeRCLCreateServiceHandleType native_rcl_create_service_handle = null;

    [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
    internal delegate int NativeRCLCreateClientHandleType (
      ref IntPtr clientHandle, IntPtr nodeHandle, [MarshalAs (UnmanagedType.LPStr)] string serviceName, IntPtr typesupportHandle);

    internal static NativeRCLCreateClientHandleType native_rcl_create_client_handle = null;

    static NodeDelegates () {
      dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils ();
      IntPtr nativelibrary = dllLoadUtils.LoadLibrary ("rcldotnet_node");

      IntPtr native_rcl_create_publisher_handle_ptr = dllLoadUtils.GetProcAddress (
        nativelibrary, "native_rcl_create_publisher_handle");

      NodeDelegates.native_rcl_create_publisher_handle =
        (NativeRCLCreatePublisherHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_publisher_handle_ptr, typeof (NativeRCLCreatePublisherHandleType));

      IntPtr native_rcl_create_subscription_handle_ptr = dllLoadUtils.GetProcAddress (
        nativelibrary, "native_rcl_create_subscription_handle");

      NodeDelegates.native_rcl_create_subscription_handle =
        (NativeRCLCreateSubscriptionHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_subscription_handle_ptr, typeof (NativeRCLCreateSubscriptionHandleType));

      IntPtr native_rcl_create_service_handle_ptr = dllLoadUtils.GetProcAddress (
        nativelibrary, "native_rcl_create_service_handle");

      NodeDelegates.native_rcl_create_service_handle =
        (NativeRCLCreateServiceHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_service_handle_ptr, typeof (NativeRCLCreateServiceHandleType));

      IntPtr native_rcl_create_client_handle_ptr = dllLoadUtils.GetProcAddress (
        nativelibrary, "native_rcl_create_client_handle");

      NodeDelegates.native_rcl_create_client_handle =
        (NativeRCLCreateClientHandleType) Marshal.GetDelegateForFunctionPointer (
          native_rcl_create_client_handle_ptr, typeof (NativeRCLCreateClientHandleType));
    }
  }

  public sealed class Node {

    private IList<Subscription> subscriptions_;

    private IList<Service> services_;

    private IList<Client> clients_;

    internal Node (IntPtr handle) {
      Handle = handle;
      subscriptions_ = new List<Subscription>();
      services_ = new List<Service>();
      clients_ = new List<Client>();
    }

    public IList<Subscription> Subscriptions => subscriptions_;

    // TODO: (sh) wrap in readonly collection
    // TODO: (sh) Add posibility to remove Subscriptions/Services/Clients from Node (if this is a thing in ROS)
    //       Now there is no Api to remove these Objects, and there is a strong reference from these fileds to them so they dont't get collected.
    public IList<Service> Services => services_;

    public IList<Client> Clients => clients_;

    public IntPtr Handle { get; }

    public Publisher<T> CreatePublisher<T> (string topic) where T : IMessage {
      MethodInfo m = typeof (T).GetTypeInfo().GetDeclaredMethod ("_GET_TYPE_SUPPORT");

      IntPtr typesupport = (IntPtr) m.Invoke (null, new object[] { });
      IntPtr publisherHandle = IntPtr.Zero;
      RCLRet ret = (RCLRet) NodeDelegates.native_rcl_create_publisher_handle (ref publisherHandle, Handle, topic, typesupport);

      // TODO: (sh) Add topic as propety to Publisher.
      Publisher<T> publisher = new Publisher<T> (publisherHandle);
      return publisher;
    }

    public Subscription<T> CreateSubscription<T> (string topic, Action<T> callback) where T : IMessage, new () {
      MethodInfo m = typeof (T).GetTypeInfo().GetDeclaredMethod ("_GET_TYPE_SUPPORT");

      IntPtr typesupport = (IntPtr) m.Invoke (null, new object[] { });
      IntPtr subscriptionHandle = IntPtr.Zero;
      RCLRet ret = (RCLRet) NodeDelegates.native_rcl_create_subscription_handle (ref subscriptionHandle, Handle, topic, typesupport);

      // TODO: (sh) Add topic as propety to Subscription.
      Subscription<T> subscription = new Subscription<T> (subscriptionHandle, callback);
      this.subscriptions_.Add(subscription);
      return subscription;
    }

    public Service<TService, TRequest, TResponse> CreateService<TService, TRequest, TResponse>(string serviceName, Action<TRequest, TResponse> callback)
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IMessage, new()
        where TResponse : IMessage, new()
    {
      MethodInfo m = typeof(TService).GetTypeInfo().GetDeclaredMethod("_GET_TYPE_SUPPORT");
      IntPtr typesupport = (IntPtr)m.Invoke (null, new object[] { });

      // TODO: (sh) check return value
      // TODO: (sh) natvie memory managment
      IntPtr serviceHandle = IntPtr.Zero;
      RCLRet ret = (RCLRet)NodeDelegates.native_rcl_create_service_handle(ref serviceHandle, Handle, serviceName, typesupport);
      
      // TODO: (sh) Add serviceName to Service.
      var service = new Service<TService, TRequest, TResponse>(serviceHandle, callback);
      this.services_.Add(service);
      return service;
    }

    public Client<TService, TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(string serviceName)
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IMessage, new()
        where TResponse : IMessage, new()
    {
      MethodInfo m = typeof(TService).GetTypeInfo().GetDeclaredMethod("_GET_TYPE_SUPPORT");
      IntPtr typesupport = (IntPtr)m.Invoke (null, new object[] { });

      // TODO: (sh) check return value
      // TODO: (sh) natvie memory managment
      IntPtr clientHandle = IntPtr.Zero;
      RCLRet ret = (RCLRet)NodeDelegates.native_rcl_create_client_handle(ref clientHandle, Handle, serviceName, typesupport);
      
      // TODO: (sh) Add serviceName to Client.
      var client = new Client<TService, TRequest, TResponse>(clientHandle, this);
      this.clients_.Add(client);
      return client;
    }
  }
}
