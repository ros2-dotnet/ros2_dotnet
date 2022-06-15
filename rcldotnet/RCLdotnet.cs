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
using ROS2.Utils;

namespace ROS2
{
    internal static class RCLdotnetDelegates
    {
        internal static readonly DllLoadUtils dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLInitType();

        internal static NativeRCLInitType native_rcl_init = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRCLGetErrorStringType(
            byte[] buffer, int bufferSize);

        internal static NativeRCLGetErrorStringType native_rcl_get_error_string = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void NativeRCLResetErrorType();

        internal static NativeRCLResetErrorType native_rcl_reset_error = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate bool NativeRCLOkType();

        internal static NativeRCLOkType native_rcl_ok = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal delegate RCLRet NativeRCLCreateNodeHandleType(
            ref SafeNodeHandle nodeHandle, [MarshalAs(UnmanagedType.LPStr)] string nodeName, [MarshalAs(UnmanagedType.LPStr)] string nodeNamespace);

        internal static NativeRCLCreateNodeHandleType native_rcl_create_node_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal delegate RCLRet NativeRCLDestroyNodeHandleType(
            IntPtr nodeHandle);

        internal static NativeRCLDestroyNodeHandleType native_rcl_destroy_node_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr NativeRCLGetRMWIdentifierType();

        internal static NativeRCLGetRMWIdentifierType native_rcl_get_rmw_identifier = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateWaitSetHandleType(
            ref SafeWaitSetHandle waitSetHandle, int numberOfSubscriptions,
            int numberOfGuardConditions, int numberOfTimers,
            int numberOfClients, int numberOfServices, int numberOfEvents);

        internal static NativeRCLCreateWaitSetHandleType native_rcl_create_wait_set_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyWaitSetType(IntPtr waitSetHandle);

        internal static NativeRCLDestroyWaitSetType native_rcl_destroy_wait_set_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetClearType(SafeWaitSetHandle waitSetHandle);

        internal static NativeRCLWaitSetClearType native_rcl_wait_set_clear = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddSubscriptionType(SafeWaitSetHandle waitSetHandle, SafeSubscriptionHandle subscriptionHandle);

        internal static NativeRCLWaitSetAddSubscriptionType native_rcl_wait_set_add_subscription = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddServiceType(SafeWaitSetHandle waitSetHandle, SafeServiceHandle serviceHandle);

        internal static NativeRCLWaitSetAddServiceType native_rcl_wait_set_add_service = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitSetAddClientType(SafeWaitSetHandle waitSetHandle, SafeClientHandle clientHandle);

        internal static NativeRCLWaitSetAddClientType native_rcl_wait_set_add_client = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLWaitType(SafeWaitSetHandle waitSetHandle, long timeout);

        internal static NativeRCLWaitType native_rcl_wait = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTakeType(SafeSubscriptionHandle subscriptionHandle, SafeHandle messageHandle);

        internal static NativeRCLTakeType native_rcl_take = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLCreateRequestIdHandleType(
            ref SafeRequestIdHandle requestIdHandle);

        internal static NativeRCLCreateRequestIdHandleType native_rcl_create_request_id_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLDestroyRequestIdHandleType(IntPtr requestIdHandle);

        internal static NativeRCLDestroyRequestIdHandleType native_rcl_destroy_request_id_handle = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate long NativeRCLRequestIdGetSequenceNumberType(SafeRequestIdHandle requestIdHandle);

        internal static NativeRCLRequestIdGetSequenceNumberType native_rcl_request_id_get_sequence_number = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTakeRequestType(SafeServiceHandle serviceHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle requestHandle);

        internal static NativeRCLTakeRequestType native_rcl_take_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLSendResponseType(SafeServiceHandle serviceHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle responseHandle);

        internal static NativeRCLSendResponseType native_rcl_send_response = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLTakeResponseType(SafeClientHandle clientHandle, SafeRequestIdHandle requestHeaderHandle, SafeHandle responseHandle);

        internal static NativeRCLTakeResponseType native_rcl_take_response = null;

        static RCLdotnetDelegates()
        {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            string library_name = "rcldotnet";
            IntPtr pDll = dllLoadUtils.LoadLibrary(library_name);

            IntPtr native_rcl_init_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_init");
            RCLdotnetDelegates.native_rcl_init =
                (NativeRCLInitType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_init_ptr, typeof(NativeRCLInitType));

            IntPtr native_rcl_get_error_string_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_get_error_string");
            RCLdotnetDelegates.native_rcl_get_error_string =
                (NativeRCLGetErrorStringType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_get_error_string_ptr, typeof(NativeRCLGetErrorStringType));

            IntPtr native_rcl_reset_error_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_reset_error");
            RCLdotnetDelegates.native_rcl_reset_error =
                (NativeRCLResetErrorType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_reset_error_ptr, typeof(NativeRCLResetErrorType));

            IntPtr native_rcl_get_rmw_identifier_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_get_rmw_identifier");
            RCLdotnetDelegates.native_rcl_get_rmw_identifier =
                (NativeRCLGetRMWIdentifierType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_get_rmw_identifier_ptr, typeof(NativeRCLGetRMWIdentifierType));

            IntPtr native_rcl_ok_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_ok");
            RCLdotnetDelegates.native_rcl_ok =
                (NativeRCLOkType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_ok_ptr, typeof(NativeRCLOkType));

            IntPtr native_rcl_create_node_handle_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_create_node_handle");
            RCLdotnetDelegates.native_rcl_create_node_handle =
                (NativeRCLCreateNodeHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_node_handle_ptr, typeof(NativeRCLCreateNodeHandleType));

            IntPtr native_rcl_destroy_node_handle_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_destroy_node_handle");
            RCLdotnetDelegates.native_rcl_destroy_node_handle =
                (NativeRCLDestroyNodeHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_node_handle_ptr, typeof(NativeRCLDestroyNodeHandleType));

            IntPtr native_rcl_create_wait_set_handle_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_create_wait_set_handle");
            RCLdotnetDelegates.native_rcl_create_wait_set_handle =
                (NativeRCLCreateWaitSetHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_wait_set_handle_ptr, typeof(NativeRCLCreateWaitSetHandleType));

            IntPtr native_rcl_destroy_wait_set_handle_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_destroy_wait_set_handle");
            RCLdotnetDelegates.native_rcl_destroy_wait_set_handle =
                (NativeRCLDestroyWaitSetType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_wait_set_handle_ptr, typeof(NativeRCLDestroyWaitSetType));

            IntPtr native_rcl_wait_set_clear_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_wait_set_clear");
            RCLdotnetDelegates.native_rcl_wait_set_clear =
                (NativeRCLWaitSetClearType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_clear_ptr, typeof(NativeRCLWaitSetClearType));

            IntPtr native_rcl_wait_set_add_subscription_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_wait_set_add_subscription");
            RCLdotnetDelegates.native_rcl_wait_set_add_subscription =
                (NativeRCLWaitSetAddSubscriptionType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_subscription_ptr, typeof(NativeRCLWaitSetAddSubscriptionType));

            IntPtr native_rcl_wait_set_add_service_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_wait_set_add_service");
            RCLdotnetDelegates.native_rcl_wait_set_add_service =
                (NativeRCLWaitSetAddServiceType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_service_ptr, typeof(NativeRCLWaitSetAddServiceType));

            IntPtr native_rcl_wait_set_add_client_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_wait_set_add_client");
            RCLdotnetDelegates.native_rcl_wait_set_add_client =
                (NativeRCLWaitSetAddClientType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_set_add_client_ptr, typeof(NativeRCLWaitSetAddClientType));

            IntPtr native_rcl_wait_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_wait");
            RCLdotnetDelegates.native_rcl_wait =
                (NativeRCLWaitType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_wait_ptr, typeof(NativeRCLWaitType));

            IntPtr native_rcl_take_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_take");
            RCLdotnetDelegates.native_rcl_take =
                (NativeRCLTakeType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_take_ptr, typeof(NativeRCLTakeType));

            IntPtr native_rcl_create_request_id_handle_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_create_request_id_handle");
            RCLdotnetDelegates.native_rcl_create_request_id_handle =
                (NativeRCLCreateRequestIdHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_create_request_id_handle_ptr, typeof(NativeRCLCreateRequestIdHandleType));

            IntPtr native_rcl_destroy_request_id_handle_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_destroy_request_id_handle");
            RCLdotnetDelegates.native_rcl_destroy_request_id_handle =
                (NativeRCLDestroyRequestIdHandleType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_destroy_request_id_handle_ptr, typeof(NativeRCLDestroyRequestIdHandleType));

            IntPtr native_rcl_request_id_get_sequence_number_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_request_id_get_sequence_number");
            RCLdotnetDelegates.native_rcl_request_id_get_sequence_number =
                (NativeRCLRequestIdGetSequenceNumberType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_request_id_get_sequence_number_ptr, typeof(NativeRCLRequestIdGetSequenceNumberType));

            IntPtr native_rcl_take_request_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_take_request");
            RCLdotnetDelegates.native_rcl_take_request =
                (NativeRCLTakeRequestType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_take_request_ptr, typeof(NativeRCLTakeRequestType));

            IntPtr native_rcl_send_response_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_send_response");
            RCLdotnetDelegates.native_rcl_send_response =
                (NativeRCLSendResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_send_response_ptr, typeof(NativeRCLSendResponseType));

            IntPtr native_rcl_take_response_ptr =
                dllLoadUtils.GetProcAddress(pDll, "native_rcl_take_response");
            RCLdotnetDelegates.native_rcl_take_response =
                (NativeRCLTakeResponseType)Marshal.GetDelegateForFunctionPointer(
                    native_rcl_take_response_ptr, typeof(NativeRCLTakeResponseType));
        }
    }

    public static class RCLdotnet
    {
        private static bool initialized = false;
        private static readonly object syncLock = new object();

        public static bool Ok()
        {
            return RCLdotnetDelegates.native_rcl_ok();
        }

        public static Node CreateNode(string nodeName)
        {
            return CreateNode(nodeName, "");
        }

        public static Node CreateNode(string nodeName, string nodeNamespace)
        {
            var nodeHandle = new SafeNodeHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_node_handle(ref nodeHandle, nodeName, nodeNamespace);
            if (ret != RCLRet.Ok)
            {
                nodeHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_node_handle)}() failed.");
            }

            Node node = new Node(nodeHandle);
            return node;
        }

        public static void Spin(Node node)
        {
            while (Ok())
            {
                SpinOnce(node, 500);
            }
        }

        private static SafeWaitSetHandle CreateWaitSet(
            int numberOfSubscriptions,
            int numberOfGuardConditions,
            int numberOfTimers,
            int numberOfClients,
            int numberOfServices,
            int numberOfEvents)
        {
            var waitSetHandle = new SafeWaitSetHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_wait_set_handle(
              ref waitSetHandle, numberOfSubscriptions, numberOfGuardConditions,
              numberOfTimers, numberOfClients, numberOfServices, numberOfEvents);
            if (ret != RCLRet.Ok)
            {
                waitSetHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_wait_set_handle)}() failed.");
            }

            return waitSetHandle;
        }

        private static void WaitSetClear(SafeWaitSetHandle waitSetHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_clear(waitSetHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_clear)}() failed.");
        }

        private static void WaitSetAddSubscription(SafeWaitSetHandle waitSetHandle, SafeSubscriptionHandle subscriptionHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_subscription(waitSetHandle, subscriptionHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_subscription)}() failed.");
        }

        private static void WaitSetAddService(SafeWaitSetHandle waitSetHandle, SafeServiceHandle serviceHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_service(waitSetHandle, serviceHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_service)}() failed.");
        }

        private static void WaitSetAddClient(SafeWaitSetHandle waitSetHandle, SafeClientHandle clientHandle)
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait_set_add_client(waitSetHandle, clientHandle);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait_set_add_client)}() failed.");
        }

        /// <summary>
        /// Block until the wait set is ready or until the timeout has been exceeded.
        /// </summary>
        /// <param name="waitSetHandle">The wait set.</param>
        /// <param name="timeout">Timeout in ms.</param>
        /// <returns>True if wait set is ready, False on timeout.</returns>
        private static bool Wait(SafeWaitSetHandle waitSetHandle, long timeout)
        {
            long ns_timeout = timeout * 1000000;
            RCLRet ret = RCLdotnetDelegates.native_rcl_wait(waitSetHandle, ns_timeout);
            if (ret == RCLRet.Ok || ret == RCLRet.Timeout)
            {
                return ret == RCLRet.Ok;
            }
            else
            {
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_wait)}() failed.");
            }
        }

        private static SafeRequestIdHandle CreateRequestId()
        {
            var requestIdHandle = new SafeRequestIdHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_request_id_handle(ref requestIdHandle);
            if (ret != RCLRet.Ok)
            {
                requestIdHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_request_id_handle)}() failed.");
            }

            return requestIdHandle;
        }

        private static bool Take(Subscription subscription, IRosMessage message)
        {
            using (var messageHandle = subscription.CreateMessageHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_take(subscription.Handle, messageHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywere.
                            // It's not worth it, especialy considering the extra allocations for SafeHandles in
                            // arrays or collections that don't realy represent their own native recource.
                            messageHandle.DangerousAddRef(ref mustRelease);
                            message.__ReadFromHandle(messageHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                messageHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.SubscriptionTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_take)}() failed.");
                }
            }
        }

        private static bool TakeRequest(Service service, SafeRequestIdHandle requestHeaderHandle, IRosMessage request)
        {
            using (var requestHandle = service.CreateRequestHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_take_request(service.Handle, requestHeaderHandle, requestHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywere.
                            // It's not worth it, especialy considering the extra allocations for SafeHandles in
                            // arrays or collections that don't realy represent their own native recource.
                            requestHandle.DangerousAddRef(ref mustRelease);
                            request.__ReadFromHandle(requestHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                requestHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ServiceTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_take_request)}() failed.");
                }
            }
        }

        private static bool TakeResponse(Client client, SafeRequestIdHandle requestHeaderHandle, IRosMessage response)
        {
            using (var responseHandle = client.CreateResponseHandle())
            {
                RCLRet ret = RCLdotnetDelegates.native_rcl_take_response(client.Handle, requestHeaderHandle, responseHandle);
                switch (ret)
                {
                    case RCLRet.Ok:
                        bool mustRelease = false;
                        try
                        {
                            // Using SafeHandles for __ReadFromHandle() is very tedious as this needs to be
                            // handled in generated code across multiple assemblies.
                            // Array and collection indexing would need to create SafeHandles everywere.
                            // It's not worth it, especialy considering the extra allocations for SafeHandles in
                            // arrays or collections that don't realy represent their own native recource.
                            responseHandle.DangerousAddRef(ref mustRelease);
                            response.__ReadFromHandle(responseHandle.DangerousGetHandle());
                        }
                        finally
                        {
                            if (mustRelease)
                            {
                                responseHandle.DangerousRelease();
                            }
                        }

                        return true;

                    case RCLRet.ClientTakeFailed:
                        return false;

                    default:
                        throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_take_response)}() failed.");
                }
            }
        }

        private static void SendResponse(Service service, SafeRequestIdHandle requestHeaderHandle, IRosMessage response)
        {
            using (var responseHandle = service.CreateResponseHandle())
            {
                bool mustRelease = false;
                try
                {
                    // Using SafeHandles for __WriteToHandle() is very tedious as this needs to be
                    // handled in generated code across multiple assemblies.
                    // Array and collection indexing would need to create SafeHandles everywere.
                    // It's not worth it, especialy considering the extra allocations for SafeHandles in
                    // arrays or collections that don't realy represent their own native recource.
                    responseHandle.DangerousAddRef(ref mustRelease);
                    response.__WriteToHandle(responseHandle.DangerousGetHandle());
                }
                finally
                {
                    if (mustRelease)
                    {
                        responseHandle.DangerousRelease();
                    }
                }

                RCLRet ret = RCLdotnetDelegates.native_rcl_send_response(service.Handle, requestHeaderHandle, responseHandle);
                if (ret != RCLRet.Ok)
                {
                    throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_send_response)}() failed.");
                }
            }
        }

        public static void SpinOnce(Node node, long timeout)
        {
            int numberOfSubscriptions = node.Subscriptions.Count;
            int numberOfGuardConditions = 0;
            int numberOfTimers = 0;
            int numberOfClients = node.Clients.Count;
            int numberOfServices = node.Services.Count;
            int numberOfEvents = 0;

            bool waitSetEmpty = numberOfSubscriptions == 0
                && numberOfGuardConditions == 0
                && numberOfTimers == 0
                && numberOfClients == 0
                && numberOfServices == 0
                && numberOfEvents == 0;

            if (waitSetEmpty)
            {
                // TODO: (sh) Should we sleep here for the timeout to avoid loops without sleep?
                // Avoid WaitSetEmpty return value from rcl_wait.
                return;
            }

            // TODO: (sh) Reuse wait set (see executer in rclc, rclcpp and rclpy)
            using (SafeWaitSetHandle waitSetHandle = CreateWaitSet(
                numberOfSubscriptions,
                numberOfGuardConditions,
                numberOfTimers,
                numberOfClients,
                numberOfServices,
                numberOfEvents))
            {
                WaitSetClear(waitSetHandle);

                foreach (Subscription subscription in node.Subscriptions)
                {
                    WaitSetAddSubscription(waitSetHandle, subscription.Handle);
                }

                foreach (var service in node.Services)
                {
                    WaitSetAddService(waitSetHandle, service.Handle);
                }

                foreach (var client in node.Clients)
                {
                    WaitSetAddClient(waitSetHandle, client.Handle);
                }

                bool ready = Wait(waitSetHandle, timeout);
                if (!ready)
                {
                    return; // timeout
                }
            }

            foreach (Subscription subscription in node.Subscriptions)
            {
                IRosMessage message = subscription.CreateMessage();
                bool result = Take(subscription, message);
                if (result)
                {
                    subscription.TriggerCallback(message);
                }
            }

            // requestIdHandle gets reused for each element in the loop.
            using (SafeRequestIdHandle requestIdHandle = CreateRequestId())
            {
                foreach (var service in node.Services)
                {
                    var request = service.CreateRequest();
                    var response = service.CreateResponse();

                    var result = TakeRequest(service, requestIdHandle, request);
                    if (result)
                    {
                        service.TriggerCallback(request, response);

                        SendResponse(service, requestIdHandle, response);
                    }
                }

                foreach (var client in node.Clients)
                {
                    var response = client.CreateResponse();

                    var result = TakeResponse(client, requestIdHandle, response);
                    if (result)
                    {
                        var sequenceNumber = RCLdotnetDelegates.native_rcl_request_id_get_sequence_number(requestIdHandle);
                        client.HandleResponse(sequenceNumber, response);
                    }
                }
            }
        }

        public static void Init()
        {
            lock (syncLock)
            {
                if (!initialized)
                {
                    RCLRet ret = RCLdotnetDelegates.native_rcl_init();
                    RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_init)}() failed.");
                    initialized = true;
                }
            }
        }

        public static string GetRMWIdentifier()
        {
            IntPtr ptr = RCLdotnetDelegates.native_rcl_get_rmw_identifier();
            string rmw_identifier = Marshal.PtrToStringAnsi(ptr);
            return rmw_identifier;
        }
    }
}
