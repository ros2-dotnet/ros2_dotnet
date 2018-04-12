using ROS2.Interfaces;

namespace ROS2 {
    public interface ISubscription : IDisposable {
        IMessage CreateMessage();
        void CallbackFN(IMessage message);
    }
}