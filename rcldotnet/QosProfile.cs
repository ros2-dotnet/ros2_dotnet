

namespace ROS2.Utils
{
    public static class QosProfile
    {
        /// <summary> QoS presets that encompass the majority of use-cases for ROS2 messages
        /// for more details on the different policies and profile use cases, see:
        /// https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/ </summary>
        public enum Profile 
        {
            /// <summary> Prioritizes timely readings, rather than 
            /// ensuring they all arrive. Best effort reliability and smaller queue depth. </summary>
            SensorData = 0,
            /// <summary> For use in services. Has large queue depth so that requests are not lost
            /// when waiting for client/server to respond. </summary>
            ProfileParameters = 1,
            /// <summary> The most similar profile to ROS1 behavior. Reliable, volatile durability, "keep last" history. </summary>
            Default = 2,
            /// <summary> For use in services. Reliable, with volatile durability to
            /// ensure that service servers who restart don't recieve outdated requests </summary>
            ServicesDefault = 3,
            /// <summary> Has a large queue depth to ensure requests are not lost. </summary>
            ParameterEvents = 4,
            /// <summary> Uses the RMW implementation's default values. </summary>
            SystemDefaults = 5,
        }
    }
}
