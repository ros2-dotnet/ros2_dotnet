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
using System.Runtime.InteropServices;
using ROS2.Utils;

namespace ROS2
{
    internal static class NodeDelegates
    {
        private static readonly DllLoadUtils _dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreatePublisherHandleType(
            ref SafePublisherHandle publisherHandle, SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string nodeName, IntPtr typesupportHandle, SafeQosProfileHandle qosProfileHandle);

        internal static NativeRCLCreatePublisherHandleType native_rcl_create_publisher_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyPublisherHandleType(
            IntPtr publisherHandle, SafeNodeHandle nodeHandle);

        internal static NativeRCLDestroyPublisherHandleType native_rcl_destroy_publisher_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateSubscriptionHandleType(
            ref SafeSubscriptionHandle subscriptionHandle, SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string nodeName, IntPtr typesupportHandle, SafeQosProfileHandle qosProfileHandle);

        internal static NativeRCLCreateSubscriptionHandleType native_rcl_create_subscription_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroySubscriptionHandleType(
            IntPtr subscriptionHandle, SafeNodeHandle nodeHandle);

        internal static NativeRCLDestroySubscriptionHandleType native_rcl_destroy_subscription_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateServiceHandleType(
            ref SafeServiceHandle serviceHandle, SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string serviceName, IntPtr typesupportHandle);

        internal static NativeRCLCreateServiceHandleType native_rcl_create_service_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyServiceHandleType(
            IntPtr serviceHandle, SafeNodeHandle nodeHandle);

        internal static NativeRCLDestroyServiceHandleType native_rcl_destroy_service_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateClientHandleType(
            ref SafeClientHandle clientHandle, SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string serviceName, IntPtr typesupportHandle);

        internal static NativeRCLCreateClientHandleType native_rcl_create_client_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyClientHandleType(
            IntPtr clientHandle, SafeNodeHandle nodeHandle);

        internal static NativeRCLDestroyClientHandleType native_rcl_destroy_client_handle = null;

        static NodeDelegates()
        {
            _dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativeLibrary = _dllLoadUtils.LoadLibrary("rcldotnet");

            IntPtr native_rcl_create_publisher_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_create_publisher_handle");

            NodeDelegates.native_rcl_create_publisher_handle =
                (NativeRCLCreatePublisherHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_publisher_handle_ptr, typeof(NativeRCLCreatePublisherHandleType));

            IntPtr native_rcl_destroy_publisher_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_destroy_publisher_handle");

            NodeDelegates.native_rcl_destroy_publisher_handle =
                (NativeRCLDestroyPublisherHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_publisher_handle_ptr, typeof(NativeRCLDestroyPublisherHandleType));

            IntPtr native_rcl_create_subscription_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_create_subscription_handle");

            NodeDelegates.native_rcl_create_subscription_handle =
                (NativeRCLCreateSubscriptionHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_subscription_handle_ptr, typeof(NativeRCLCreateSubscriptionHandleType));

            IntPtr native_rcl_destroy_subscription_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_destroy_subscription_handle");

            NodeDelegates.native_rcl_destroy_subscription_handle =
                (NativeRCLDestroySubscriptionHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_subscription_handle_ptr, typeof(NativeRCLDestroySubscriptionHandleType));

            IntPtr native_rcl_create_service_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_create_service_handle");

            NodeDelegates.native_rcl_create_service_handle =
                (NativeRCLCreateServiceHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_service_handle_ptr, typeof(NativeRCLCreateServiceHandleType));

            IntPtr native_rcl_destroy_service_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_destroy_service_handle");

            NodeDelegates.native_rcl_destroy_service_handle =
                (NativeRCLDestroyServiceHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_service_handle_ptr, typeof(NativeRCLDestroyServiceHandleType));

            IntPtr native_rcl_create_client_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_create_client_handle");

            NodeDelegates.native_rcl_create_client_handle =
                (NativeRCLCreateClientHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_client_handle_ptr, typeof(NativeRCLCreateClientHandleType));

            IntPtr native_rcl_destroy_client_handle_ptr = _dllLoadUtils.GetProcAddress(
                nativeLibrary, "native_rcl_destroy_client_handle");

            NodeDelegates.native_rcl_destroy_client_handle =
                (NativeRCLDestroyClientHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_client_handle_ptr, typeof(NativeRCLDestroyClientHandleType));
        }
    }

    public sealed class Node
    {
        private readonly IList<Subscription> _subscriptions;

        private readonly IList<Service> _services;

        private readonly IList<Client> _clients;

        private readonly IList<GuardCondition> _guardConditions;

        internal Node(SafeNodeHandle handle)
        {
            Handle = handle;
            _subscriptions = new List<Subscription>();
            _services = new List<Service>();
            _clients = new List<Client>();
            _guardConditions = new List<GuardCondition>();
        }

        public IList<Subscription> Subscriptions => _subscriptions;

        // TODO: (sh) wrap in readonly collection
        // TODO: (sh) Add posibility to remove Subscriptions/Services/Clients from Node (if this is a thing in ROS)
        //       Now there is no Api to remove these Objects, and there is a strong reference from these fileds to them so they dont't get collected.
        public IList<Service> Services => _services;

        public IList<Client> Clients => _clients;

        public IList<GuardCondition> GuardConditions => _guardConditions;

        // Node does intentionaly (for now) not implement IDisposable as this
        // needs some extra consideration how the type works after its
        // internal handle is disposed.
        // By relying on the GC/Finalizer of SafeHandle the handle only gets
        // Disposed if the node is not live anymore.
        internal SafeNodeHandle Handle { get; }

        public Publisher<T> CreatePublisher<T>(string topic, QosProfile qosProfile = null) where T : IRosMessage
        {
            if (qosProfile != null)
            {
                using (SafeQosProfileHandle qosProfileHandle = QosProfile.CreateQosProfileHandle())
                {
                    QosProfile.WriteToQosProfileHandle(qosProfile, qosProfileHandle);
                    return CreatePublisherInner<T>(topic, qosProfileHandle);
                }
            }
            else
            {
                return CreatePublisherInner<T>(topic, SafeQosProfileHandle.Null);
            }
        }

        private Publisher<T> CreatePublisherInner<T>(string topic, SafeQosProfileHandle qosProfileHandle) where T : IRosMessage
        {
            IntPtr typeSupport = MessageStaticMemberCache<T>.GetTypeSupport();

            var publisherHandle = new SafePublisherHandle();
            RCLRet ret = NodeDelegates.native_rcl_create_publisher_handle(ref publisherHandle, Handle, topic, typeSupport, qosProfileHandle);
            publisherHandle.SetParent(Handle);
            if (ret != RCLRet.Ok)
            {
                publisherHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(NodeDelegates.native_rcl_create_publisher_handle)}() failed.");
            }

            // TODO: (sh) Add topic as propety to Publisher.
            Publisher<T> publisher = new Publisher<T>(publisherHandle);
            return publisher;
        }

        public Subscription<T> CreateSubscription<T>(string topic, Action<T> callback, QosProfile qosProfile = null) where T : IRosMessage, new()
        {
            if (qosProfile != null)
            {
                using (SafeQosProfileHandle qosProfileHandle = QosProfile.CreateQosProfileHandle())
                {
                    QosProfile.WriteToQosProfileHandle(qosProfile, qosProfileHandle);
                    return CreateSubscriptionInner(topic, callback, qosProfileHandle);
                }
            }
            else
            {
                return CreateSubscriptionInner(topic, callback, SafeQosProfileHandle.Null);
            }
        }

        private Subscription<T> CreateSubscriptionInner<T>(string topic, Action<T> callback, SafeQosProfileHandle qosProfileHandle) where T : IRosMessage, new()
        {
            IntPtr typeSupport = MessageStaticMemberCache<T>.GetTypeSupport();

            var subscriptionHandle = new SafeSubscriptionHandle();
            RCLRet ret = NodeDelegates.native_rcl_create_subscription_handle(ref subscriptionHandle, Handle, topic, typeSupport, qosProfileHandle);
            subscriptionHandle.SetParent(Handle);
            if (ret != RCLRet.Ok)
            {
                subscriptionHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(NodeDelegates.native_rcl_create_subscription_handle)}() failed.");
            }

            // TODO: (sh) Add topic as propety to Subscription.
            Subscription<T> subscription = new Subscription<T>(subscriptionHandle, callback);
            _subscriptions.Add(subscription);
            return subscription;
        }

        public Service<TService, TRequest, TResponse> CreateService<TService, TRequest, TResponse>(string serviceName, Action<TRequest, TResponse> callback)
            where TService : IRosServiceDefinition<TRequest, TResponse>
            where TRequest : IRosMessage, new()
            where TResponse : IRosMessage, new()
        {
            IntPtr typeSupport = ServiceDefinitionStaticMemberCache<TService, TRequest, TResponse>.GetTypeSupport();

            var serviceHandle = new SafeServiceHandle();
            RCLRet ret = NodeDelegates.native_rcl_create_service_handle(ref serviceHandle, Handle, serviceName, typeSupport);
            serviceHandle.SetParent(Handle);
            if (ret != RCLRet.Ok)
            {
                serviceHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(NodeDelegates.native_rcl_create_service_handle)}() failed.");
            }

            // TODO: (sh) Add serviceName to Service.
            var service = new Service<TService, TRequest, TResponse>(serviceHandle, callback);
            _services.Add(service);
            return service;
        }

        public Client<TService, TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(string serviceName)
            where TService : IRosServiceDefinition<TRequest, TResponse>
            where TRequest : IRosMessage, new()
            where TResponse : IRosMessage, new()
        {
            IntPtr typeSupport = ServiceDefinitionStaticMemberCache<TService, TRequest, TResponse>.GetTypeSupport();

            var clientHandle = new SafeClientHandle();
            RCLRet ret = NodeDelegates.native_rcl_create_client_handle(ref clientHandle, Handle, serviceName, typeSupport);
            clientHandle.SetParent(Handle);
            if (ret != RCLRet.Ok)
            {
                clientHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(NodeDelegates.native_rcl_create_client_handle)}() failed.");
            }

            // TODO: (sh) Add serviceName to Client.
            var client = new Client<TService, TRequest, TResponse>(clientHandle, this);
            _clients.Add(client);
            return client;
        }

        public GuardCondition CreateGuardCondition(Action callback)
        {
            var guardConditionHandle = new SafeGuardConditionHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_guard_condition_handle(ref guardConditionHandle);
            if (ret != RCLRet.Ok)
            {
                guardConditionHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_guard_condition_handle)}() failed.");
            }

            var guardCondition = new GuardCondition(guardConditionHandle, callback);
            _guardConditions.Add(guardCondition);
            return guardCondition;
        }

        public ActionClient<TAction, TGoal, TResult, TFeedback> CreateActionClient<TAction, TGoal, TResult, TFeedback>(string actionName)
            where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
            where TGoal : IRosMessage, new()
            where TResult : IRosMessage, new()
            where TFeedback : IRosMessage, new()
        {
            throw new NotImplementedException();
        }
    }
}
