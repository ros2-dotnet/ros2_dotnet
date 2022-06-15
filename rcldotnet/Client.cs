/* Copyright 2021-2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using ROS2.Utils;

namespace ROS2
{
    internal static class ClientDelegates
    {
        internal static readonly DllLoadUtils dllLoadUtils;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLSendRequestType(
            SafeClientHandle clientHandle, SafeHandle requestHandle, out long seqneceNumber);

        internal static NativeRCLSendRequestType native_rcl_send_request = null;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate RCLRet NativeRCLServiceServerIsAvailableType(
            SafeNodeHandle nodeHandle, SafeClientHandle clientHandle, out bool isAvailable);

        internal static NativeRCLServiceServerIsAvailableType native_rcl_service_server_is_available = null;

        static ClientDelegates()
        {
            dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
            IntPtr nativelibrary = dllLoadUtils.LoadLibrary("rcldotnet_client");

            IntPtr native_rcl_send_request_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_rcl_send_request");
            ClientDelegates.native_rcl_send_request = (NativeRCLSendRequestType)Marshal.GetDelegateForFunctionPointer(
                native_rcl_send_request_ptr, typeof(NativeRCLSendRequestType));

            IntPtr native_rcl_service_server_is_available_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "native_rcl_service_server_is_available");
            ClientDelegates.native_rcl_service_server_is_available = (NativeRCLServiceServerIsAvailableType)Marshal.GetDelegateForFunctionPointer(
                native_rcl_service_server_is_available_ptr, typeof(NativeRCLServiceServerIsAvailableType));
        }
    }

    /// <summary>
    /// Base class of a Client without generic type arguments for use in collections or so.
    /// </summary>
    public abstract class Client
    {
        // Only allow internal subclasses.
        internal Client()
        {
        }

        // Client does intentionaly (for now) not implement IDisposable as this
        // needs some extra consideration how the type works after its
        // internal handle is disposed.
        // By relying on the GC/Finalizer of SafeHandle the handle only gets
        // Disposed if the client is not live anymore.
        internal abstract SafeClientHandle Handle { get; }

        internal abstract IRosMessage CreateResponse();

        internal abstract SafeHandle CreateResponseHandle();

        internal abstract void HandleResponse(long sequenceNumber, IRosMessage response);
    }

    public sealed class Client<TService, TRequest, TResponse> : Client
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IRosMessage, new()
        where TResponse : IRosMessage, new()
    {
        // ros2_java uses a WeakReference here. Not sure if its needed or not.
        private readonly Node _node;
        private readonly ConcurrentDictionary<long, PendingRequest> _pendingRequests = new ConcurrentDictionary<long, PendingRequest>();

        internal Client(SafeClientHandle handle, Node node)
        {
            Handle = handle;
            _node = node;
        }

        internal override SafeClientHandle Handle { get; }

        public bool ServiceIsReady()
        {
            RCLRet ret = ClientDelegates.native_rcl_service_server_is_available(_node.Handle, Handle, out var serviceIsReady);
            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClientDelegates.native_rcl_service_server_is_available)}() failed.");

            return serviceIsReady;
        }

        public Task<TResponse> SendRequestAsync(TRequest request)
        {
            // TODO: (sh) Add cancellationToken(?), timeout (via cancellationToken?) and cleanup of pending requests.
            long sequenceNumber;

            using (var requestHandle = MessageStaticMemberCache<TRequest>.CreateMessageHandle())
            {
                bool mustRelease = false;
                try
                {
                    // Using SafeHandles for __WriteToHandle() is very tedious as this needs to be
                    // handled in generated code across multiple assemblies.
                    // Array and collection indexing would need to create SafeHandles everywere.
                    // It's not worth it, especialy considering the extra allocations for SafeHandles in
                    // arrays or collections that don't realy represent their own native recource.
                    requestHandle.DangerousAddRef(ref mustRelease);
                    request.__WriteToHandle(requestHandle.DangerousGetHandle());
                }
                finally
                {
                    if (mustRelease)
                    {
                        requestHandle.DangerousRelease();
                    }
                }

                RCLRet ret = ClientDelegates.native_rcl_send_request(Handle, requestHandle, out sequenceNumber);
                RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(ClientDelegates.native_rcl_send_request)}() failed.");
            }

            var taskCompletionSource = new TaskCompletionSource<TResponse>();
            var pendingRequest = new PendingRequest(taskCompletionSource);
            if (!_pendingRequests.TryAdd(sequenceNumber, pendingRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber already taken.
            }

            return taskCompletionSource.Task;
        }

        internal override IRosMessage CreateResponse() => (IRosMessage)new TResponse();

        internal override SafeHandle CreateResponseHandle() => MessageStaticMemberCache<TResponse>.CreateMessageHandle();

        internal override void HandleResponse(long sequenceNumber, IRosMessage response)
        {
            if (!_pendingRequests.TryRemove(sequenceNumber, out var pendingRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber not found.
                return;
            }

            pendingRequest.TaskCompletionSource.TrySetResult((TResponse)response);
        }

        private sealed class PendingRequest
        {
            public PendingRequest(TaskCompletionSource<TResponse> taskCompletionSource)
            {
                TaskCompletionSource = taskCompletionSource;
            }

            public TaskCompletionSource<TResponse> TaskCompletionSource { get; }
        }
    }
}
