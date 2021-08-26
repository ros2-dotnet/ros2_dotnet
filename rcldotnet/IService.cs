using ROS2.Interfaces;

namespace ROS2
{
    public interface IService<TService, TRequest, TResponse> : IServiceBase
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IMessage, new()
        where TResponse : IMessage, new()
    {
    }
}
