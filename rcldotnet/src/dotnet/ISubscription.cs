using System;
using ROS2.Interfaces;

namespace rclcs
{
    public interface ISubscription<T>: ISubscriptionBase
    where T: IRclcsMessage {}
}
