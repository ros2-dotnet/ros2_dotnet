using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using ROS2.Interfaces;
using ROS2.Utils;

namespace ROS2
{
    public sealed class Service<TService, TRequest, TResponse> : IService<TService, TRequest, TResponse>
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IMessage, new()
        where TResponse : IMessage, new()
    {
        private Action<TRequest, TResponse> _callback;

        public Service(IntPtr handle, Action<TRequest, TResponse> callback)
        {
            Handle = handle;
            _callback = callback;
        }

        public IntPtr Handle { get; }

        public IMessage CreateRequest() => (IMessage)new TRequest();

        public IMessage CreateResponse() => (IMessage)new TResponse();

        public void TriggerCallback(IMessage request, IMessage response)
        {
            _callback((TRequest)request, (TResponse)response);
        }
    }
}
