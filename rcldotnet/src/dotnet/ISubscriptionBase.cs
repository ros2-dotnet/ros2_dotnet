using ROS2.Interfaces;

namespace rclcs
{
    public interface ISubscriptionBase : System.IDisposable
    {
        rcl_subscription_t Handle { get; }
        IRclcsMessage CreateMessage();
        void TriggerCallback(IRclcsMessage message);
    }

}
