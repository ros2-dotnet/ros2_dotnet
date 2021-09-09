namespace ROS2
{
    public interface IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IRosMessage, new()
        where TResponse : IRosMessage, new()
    {
        // must be implemented on deriving types, gets called via reflection
        // (static abstract interface members are not supported yet.)
        // public static abstract IntPtr __GetTypeSupport();
    }
}
