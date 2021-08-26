using ROS2.Interfaces;

namespace ROS2
{
    public interface IServiceBase : ROS2.Interfaces.IDisposable
    {
        IMessage CreateRequest();
        
        IMessage CreateResponse();

        void TriggerCallback (IMessage request, IMessage response);
    }
}
