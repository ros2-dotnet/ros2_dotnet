using System;
using ROS2.Interfaces;

namespace ROS2
{
    /// <summary>
    /// Base class of a Service without generic type arguments for use in collections or so.
    /// </summary>
    public abstract class Service
    {
        // Only allow internal subclasses.
        internal Service()
        {
        }

        abstract internal IntPtr Handle { get; }

        abstract internal IMessage CreateRequest();

        abstract internal IMessage CreateResponse();

        abstract internal void TriggerCallback(IMessage request, IMessage response);
    }

    public sealed class Service<TService, TRequest, TResponse> : Service
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IMessage, new()
        where TResponse : IMessage, new()
    {
        private Action<TRequest, TResponse> _callback;

        internal Service(IntPtr handle, Action<TRequest, TResponse> callback)
        {
            Handle = handle;
            _callback = callback;
        }

        internal override IntPtr Handle { get; }

        internal override IMessage CreateRequest() => (IMessage)new TRequest();

        internal override IMessage CreateResponse() => (IMessage)new TResponse();

        internal override void TriggerCallback(IMessage request, IMessage response)
        {
            _callback((TRequest)request, (TResponse)response);
        }
    }
}
