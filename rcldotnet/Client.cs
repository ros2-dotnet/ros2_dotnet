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

namespace ROS2
{
    internal class ClientDelegates
    {
        internal static readonly DllLoadUtils dllLoadUtils;

        [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
        internal delegate int NativeRCLSendRequestType(
            IntPtr clientHandle, IntPtr requestHandle, out long seqneceNumber);

        internal static NativeRCLSendRequestType native_rcl_send_request = null;

        [UnmanagedFunctionPointer (CallingConvention.Cdecl)]
        internal delegate int NativeRCLServiceServerIsAvailableType(
            IntPtr nodeHandle, IntPtr clientHandle, out bool isAvailable);

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

    public abstract class Client : ROS2.Interfaces.IDisposable
    {
        public abstract IntPtr Handle { get; }

        internal abstract IMessage CreateResponse();

        internal abstract void HandleResponse(long sequenceNumber, IMessage response);
    }

    public sealed class Client<TService, TRequest, TResponse> : Client
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IMessage, new()
        where TResponse : IMessage, new()
    {
        // ros2_java uses a WeakReference here. Not sure if its needed or not.
        private readonly Node _node;
        private readonly ConcurrentDictionary<long, PendingRequest> _pendingRequests = new ConcurrentDictionary<long, PendingRequest>();
        
        internal Client(IntPtr handle, Node node)
        {
            Handle = handle;
            _node = node;
        }

        public override IntPtr Handle { get; }

        public bool ServiceIsReady()
        {
            var ret = (RCLRet)ClientDelegates.native_rcl_service_server_is_available(_node.Handle, Handle, out var serviceIsReady);
            if (ret != RCLRet.Ok)
            {
                throw new Exception($"rcl_service_server_is_available() failed: {ret}");
            }

            return serviceIsReady;
        }

        public Task<TResponse> SendRequestAsync(TRequest request)
        {
            // TODO: (sh) Add cancellationToken(?), timeout (via cancellationToken?) and cleanup of pending requests.
            // TODO: (sh) Catch all exceptions and return Task with error?
            //       How should this be done according to best practices.
            IntPtr requestHandle = request._CREATE_NATIVE_MESSAGE();

            request._WRITE_HANDLE(requestHandle);

            RCLRet ret = (RCLRet)ClientDelegates.native_rcl_send_request(Handle, requestHandle, out var sequenceNumber);

            // TODO: (sh) don't leak memory on exceptions
            request._DESTROY_NATIVE_MESSAGE(requestHandle);

            var taskCompletionSource = new TaskCompletionSource<TResponse>();
            var pendingRequest = new PendingRequest(taskCompletionSource);
            if (!_pendingRequests.TryAdd(sequenceNumber, pendingRequest))
            {
                // TODO: (sh) Add error handling/logging. SequenceNumber already taken.
            }

            return taskCompletionSource.Task;
        }

        internal override IMessage CreateResponse() => (IMessage)new TResponse();

        internal override void HandleResponse(long sequenceNumber, IMessage response)
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
