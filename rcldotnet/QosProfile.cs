/* Copyright 2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Threading;

namespace ROS2
{
    /// <summary>
    /// The `HISTORY` DDS QoS policy.
    ///
    /// A subscription internally maintains a queue of messages (called "samples" in DDS) that have not
    /// been processed yet by the application, and likewise a publisher internally maintains a queue.
    ///
    /// If the history policy is `KeepAll`, this queue is unbounded, and if it is `KeepLast`, it is
    /// bounded and old values are discarded when the queue is overfull.
    ///
    /// # Compatibility
    /// | Publisher | Subscription | Compatible |
    /// | --------- | ------------ | ---------- |
    /// | KeepLast  | KeepLast     | yes        |
    /// | KeepLast  | KeepAll      | yes        |
    /// | KeepAll   | KeepLast     | yes        |
    /// | KeepAll   | KeepAll      | yes        |
    /// </summary>
    public enum QosHistoryPolicy
    {
        /// <summary>
        /// Implementation default for history policy.
        /// </summary>
        SystemDefault = 0,

        /// <summary>
        /// Only store up to a maximum number of samples, dropping oldest once max is exceeded.
        /// </summary>
        KeepLast = 1,

        /// <summary>
        /// Store all samples, subject to resource limits.
        /// </summary>
        KeepAll = 2,
    }

    /// <summary>
    /// The `RELIABILITY` DDS QoS policy.
    ///
    /// This policy determines whether delivery between a publisher and a subscription will be retried
    /// until successful, or whether messages may be lost in a trade off for better performance.
    ///
    /// # Compatibility
    /// | Publisher  | Subscription | Compatible | Behavior    |
    /// | ---------- | ------------ | ---------- | ----------- |
    /// | Reliable   | Reliable     | yes        | Reliable    |
    /// | Reliable   | BestEffort   | yes        | Best effort |
    /// | BestEffort | Reliable     | no         | -           |
    /// | BestEffort | BestEffort   | yes        | Best effort |
    /// </summary>
    public enum QosReliabilityPolicy
    {
        /// <summary>
        /// Use the default policy of the RMW layer.
        /// </summary>
        SystemDefault = 0,

        /// <summary>
        /// Guarantee delivery of messages.
        /// </summary>
        Reliable = 1,

        /// <summary>
        /// Send messages but do not guarantee delivery.
        /// </summary>
        BestEffort = 2,
    }

    /// <summary>
    /// The `DURABILITY` DDS QoS policy.
    ///
    /// If a subscription is created after some messages have already been published, it is possible
    /// for the subscription to receive a number of previously-published messages by using the
    /// "transient local" durability kind on both ends. For this, the publisher must still exist when
    /// the subscription is created.
    ///
    /// # Compatibility
    /// | Publisher      | Subscription   | Compatible | Behavior                                  |
    /// | -------------- | -------------- | ---------- | ----------------------------------------- |
    /// | TransientLocal | TransientLocal | yes        | Deliver old messages to new subscriptions |
    /// | TransientLocal | Volatile       | yes        | Deliver only new messages                 |
    /// | Volatile       | TransientLocal | no         | -                                         |
    /// | Volatile       | Volatile       | yes        | Deliver only new messages                 |
    /// </summary>
    public enum QosDurabilityPolicy
    {
        /// <summary>
        /// Use the default policy of the RMW layer.
        /// </summary>
        SystemDefault = 0,

        /// <summary>
        /// Re-deliver old messages.
        /// - For publishers: Retain messages for later delivery.
        /// - For subscriptions: Request delivery of old messages.
        /// </summary>
        TransientLocal = 1,

        /// <summary>
        /// Do not retain/request old messages.
        /// </summary>
        Volatile = 2,
    }

    /// <summary>
    /// The `LIVELINESS` DDS QoS policy.
    ///
    /// This policy describes a publisher's reporting policy for its alive status.
    /// For a subscription, these are its requirements for its topic's publishers.
    ///
    /// # Compatibility
    /// | Publisher     | Subscription  | Compatible |
    /// | ------------- | ------------- | ---------- |
    /// | Automatic     | Automatic     | yes        |
    /// | Automatic     | ManualByTopic | no         |
    /// | ManualByTopic | Automatic     | yes        |
    /// | ManualByTopic | ManualByTopic | yes        |
    /// </summary>
    public enum QosLivelinessPolicy
    {
        /// <summary>
        /// Use the default policy of the RMW layer.
        /// </summary>
        SystemDefault = 0,

        /// <summary>
        /// The signal that establishes that a topic is alive comes from the ROS `rmw` layer.
        /// </summary>
        Automatic = 1,

        /// <summary>
        /// The signal that establishes that a topic is alive is sent explicitly. Only publishing a message
        /// on the topic or an explicit signal from the application to assert liveliness on the topic
        /// will mark the topic as being alive.
        /// </summary>
        ManualByTopic = 3,
    }

    /// <summary>
    /// A Quality of Service profile.
    ///
    /// See [docs.ros.org][1] on Quality of Service settings in general.
    ///
    /// In the general case, a topic can have multiple publishers and multiple subscriptions, each with
    /// an individual QoS profile. For each publisher-subscription pair, messages are only delivered if
    /// their QoS profiles are compatible.
    ///
    /// [1]: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
    /// </summary>
    public sealed class QosProfile
    {
        // This is private to force use of factory methods.
        private QosProfile(
            QosHistoryPolicy history,
            int depth,
            QosReliabilityPolicy reliability,
            QosDurabilityPolicy durability,
            TimeSpan deadline,
            TimeSpan lifespan,
            QosLivelinessPolicy liveliness,
            TimeSpan livelinessLeaseDuration,
            bool avoidRosNamespaceConventions)
        {
            History = history;
            Depth = depth;
            Reliability = reliability;
            Durability = durability;
            Deadline = deadline;
            Lifespan = lifespan;
            Liveliness = liveliness;
            LivelinessLeaseDuration = livelinessLeaseDuration;
            AvoidRosNamespaceConventions = avoidRosNamespaceConventions;
        }

        // RMW_DURATION_INFINITE is defined as INT64_MAX nanoseconds, which does
        // not map nicely to TimeSpan as it uses 100ns ticks as representation.
        //
        // So this uses Timeout.InfiniteTimeSpan as special value.
        // Timeout.InfiniteTimeSpan is defined as -1ms. This does not collide
        // with the rmw_time_t values as it is defined as unsigned integers for
        // seconds.

        /// <summary>
        /// Special value for TimeSpan to indicate Infinite on all TimeSpan
        /// properties fo this class.
        /// </summary>
        public static TimeSpan InfiniteDuration => Timeout.InfiniteTimeSpan;

        /// <summary>
        /// The default QoS profile.
        ///
        /// Unlikely to change but valid as at 2023-10-31.
        /// https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
        ///
        /// | --------------- | --------------- |
        /// | History         | KEEP_LAST       |
        /// | Depth           | 10              |
        /// | Reliability     | RELIABLE        |
        /// | Durability      | VOLATILE        |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile DefaultProfile => ReadConstQoSProfile(QosProfileDelegates.native_rcl_qos_get_const_profile_default);

        /// <summary>
        /// Profile for clock messages.
        /// See CreateClockProfile for more details.
        ///
        /// | --------------- | --------------- |
        /// | History         | KEEP_LAST       |
        /// | Depth           | 1               |
        /// | Reliability     | BEST_EFFORT     |
        /// | Durability      | VOLATILE        |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile ClockProfile => CreateClockProfile();

        /// <summary>
        /// Profile for parameter event messages.
        ///
        /// Unlikely to change but valid as at 2023-10-31.
        /// https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
        ///
        /// | --------------- | --------------- |
        /// | History         | KEEP_LAST       |
        /// | Depth           | 1000            |
        /// | Reliability     | RELIABLE        |
        /// | Durability      | VOLATILE        |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile ParameterEventsProfile => ReadConstQoSProfile(QosProfileDelegates.native_rcl_qos_get_const_profile_parameter_events);

        /// <summary>
        /// Profile for parameter messages.
        ///
        /// Unlikely to change but valid as at 2023-10-31.
        /// https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
        ///
        /// | --------------- | --------------- |
        /// | History         | KEEP_LAST       |
        /// | Depth           | 1000            |
        /// | Reliability     | RELIABLE        |
        /// | Durability      | VOLATILE        |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile ParametersProfile => ReadConstQoSProfile(QosProfileDelegates.native_rcl_qos_get_const_profile_parameters);

        /// <summary>
        /// Profile for sensor messages.
        ///
        /// Unlikely to change but valid as at 2023-10-31.
        /// https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
        ///
        /// | --------------- | --------------- |
        /// | History         | KEEP_LAST       |
        /// | Depth           | 5               |
        /// | Reliability     | BEST_EFFORT     |
        /// | Durability      | VOLATILE        |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile SensorDataProfile => ReadConstQoSProfile(QosProfileDelegates.native_rcl_qos_get_const_profile_sensor_data);

        /// <summary>
        /// Default profile for services.
        ///
        /// Unlikely to change but valid as at 2023-10-31.
        /// https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
        ///
        /// | --------------- | --------------- |
        /// | History         | KEEP_LAST       |
        /// | Depth           | 10              |
        /// | Reliability     | RELIABLE        |
        /// | Durability      | VOLATILE        |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile ServicesDefaultProfile => ReadConstQoSProfile(QosProfileDelegates.native_rcl_qos_get_const_profile_services_default);

        /// <summary>
        /// The system default (null) profile.
        ///
        /// Unlikely to change but valid as at 2023-10-31.
        /// https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
        ///
        /// | --------------- | --------------- |
        /// | History         | SYSTEM_DEFAULT  |
        /// | Depth           | SYSTEM_DEFAULT  |
        /// | Reliability     | SYSTEM_DEFAULT  |
        /// | Durability      | SYSTEM_DEFAULT  |
        /// | Deadline        | DEFAULT         |
        /// | Lifespan        | DEFAULT         |
        /// | Liveliness      | SYSTEM_DEFAULT  |
        /// | Lease Duration  | DEFAULT         |
        /// | Avoid Namespace | false           |
        /// </summary>
        public static QosProfile SystemDefaultProfile => ReadConstQoSProfile(QosProfileDelegates.native_rcl_qos_get_const_profile_system_default);

        /// <summary>
        /// The history policy.
        /// </summary>
        public QosHistoryPolicy History { get; }

        /// <summary>
        /// Size of the message queue.
        /// </summary>
        public int Depth { get; }

        /// <summary>
        /// The reliability policy.
        /// </summary>
        public QosReliabilityPolicy Reliability { get; }

        /// <summary>
        /// The durability policy.
        /// </summary>
        public QosDurabilityPolicy Durability { get; }

        /// <summary>
        /// The period at which messages are expected to be sent/received.
        ///
        /// If this is <see cref="QosProfile.InfiniteDuration">, messages never miss a deadline expectation.
        /// </summary>
        public TimeSpan Deadline { get; }

        /// <summary>
        /// The age at which messages are considered expired and no longer valid.
        ///
        /// If this is <see cref="QosProfile.InfiniteDuration">, messages do not expire.
        /// </summary>
        public TimeSpan Lifespan { get; }

        /// <summary>
        /// The liveliness policy.
        /// </summary>
        public QosLivelinessPolicy Liveliness { get; }

        /// <summary>
        /// The time within which the RMW publisher must show that it is alive.
        ///
        /// If this is <see cref="QosProfile.InfiniteDuration">, liveliness is not enforced.
        /// </summary>
        public TimeSpan LivelinessLeaseDuration { get; }

        /// <summary>
        /// If true, any ROS specific namespacing conventions will be circumvented.
        ///
        /// In the case of DDS and topics, for example, this means the typical
        /// ROS specific prefix of `rt` would not be applied as described [here][1].
        ///
        /// This might be useful when trying to directly connect a native DDS topic
        /// with a ROS 2 topic.
        ///
        /// [1]: http://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix
        /// </summary>
        public bool AvoidRosNamespaceConventions { get; }

        /// <summary>
        /// Create a new QosProfile with the history set to KeepLast and the given depth.
        /// </summary>
        /// <param name="depth">Size of the message queue.</param>
        public static QosProfile KeepLast(int depth)
        {
            return DefaultProfile.WithKeepLast(depth);
        }

        /// <summary>
        /// Create a new QosProfile with the history set to KeepAll.
        /// </summary>
        public static QosProfile KeepAll()
        {
            return DefaultProfile.WithKeepAll();
        }

        /// <summary>
        /// Set the history to KeepLast and the given depth.
        /// </summary>
        /// <param name="depth">Size of the message queue.</param>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithKeepLast(int depth)
        {
            return WithHistoryAndDepth(QosHistoryPolicy.KeepLast, depth);
        }

        /// <summary>
        /// Set the history to KeepAll.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithKeepAll()
        {
            return WithHistoryAndDepth(QosHistoryPolicy.KeepAll, depth: 0);
        }

        // This is private to disallow invalid combination off values.
        private QosProfile WithHistoryAndDepth(QosHistoryPolicy history, int depth)
        {
            var result = new QosProfile(
                history: history,
                depth: depth,
                reliability: Reliability,
                durability: Durability,
                deadline: Deadline,
                lifespan: Lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="Reliability">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithReliability(QosReliabilityPolicy reliability)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: reliability,
                durability: Durability,
                deadline: Deadline,
                lifespan: Lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="Reliability"> to <see cref="QosReliabilityPolicy.Reliable">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithReliable() => WithReliability(QosReliabilityPolicy.Reliable);

        /// <summary>
        /// Set the <see cref="Reliability"> to <see cref="QosReliabilityPolicy.BestEffort">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithBestEffort() => WithReliability(QosReliabilityPolicy.BestEffort);

        /// <summary>
        /// Set the <see cref="Durability">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithDurability(QosDurabilityPolicy durability)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: Reliability,
                durability: durability,
                deadline: Deadline,
                lifespan: Lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="Durability"> to <see cref="QosDurabilityPolicy.TransientLocal">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithTransientLocal() => WithDurability(QosDurabilityPolicy.TransientLocal);

        /// <summary>
        /// Set the <see cref="Volatile"> to <see cref="QosDurabilityPolicy.Volatile">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithVolatile() => WithDurability(QosDurabilityPolicy.Volatile);

        /// <summary>
        /// Set the <see cref="Deadline">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithDeadline(TimeSpan deadline)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: Reliability,
                durability: Durability,
                deadline: deadline,
                lifespan: Lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="Lifespan">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithLifespan(TimeSpan lifespan)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: Reliability,
                durability: Durability,
                deadline: Deadline,
                lifespan: lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="Liveliness">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithLiveliness(QosLivelinessPolicy liveliness)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: Reliability,
                durability: Durability,
                deadline: Deadline,
                lifespan: Lifespan,
                liveliness: liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="LivelinessLeaseDuration">.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithLivelinessLeaseDuration(TimeSpan livelinessLeaseDuration)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: Reliability,
                durability: Durability,
                deadline: Deadline,
                lifespan: Lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: livelinessLeaseDuration,
                avoidRosNamespaceConventions: AvoidRosNamespaceConventions);

            return result;
        }

        /// <summary>
        /// Set the <see cref="AvoidRosNamespaceConventions"> value.
        /// </summary>
        /// <returns>A new QosProfile with the modification.</returns>
        public QosProfile WithAvoidRosNamespaceConventions(bool avoidRosNamespaceConventions)
        {
            var result = new QosProfile(
                history: History,
                depth: Depth,
                reliability: Reliability,
                durability: Durability,
                deadline: Deadline,
                lifespan: Lifespan,
                liveliness: Liveliness,
                livelinessLeaseDuration: LivelinessLeaseDuration,
                avoidRosNamespaceConventions: avoidRosNamespaceConventions);

            return result;
        }

        private static QosProfile CreateClockProfile()
        {
            // Values from https://docs.ros.org/en/rolling/p/rclcpp/generated/classrclcpp_1_1ClockQoS.html
            // Only available in versions of rclcpp >= Galactic
            // If changed, update comment at top of file also.
            return new QosProfile(
                history: QosHistoryPolicy.KeepLast,
                depth: 1,
                reliability: QosReliabilityPolicy.BestEffort,
                durability: QosDurabilityPolicy.Volatile,
                deadline: TimeSpan.Zero,
                lifespan: TimeSpan.Zero,
                liveliness: QosLivelinessPolicy.SystemDefault,
                livelinessLeaseDuration: TimeSpan.Zero,
                avoidRosNamespaceConventions: false
            );
        }

        internal static SafeQosProfileHandle CreateQosProfileHandle()
        {
            var qosProfileHandle = new SafeQosProfileHandle();
            RCLRet ret = RCLdotnetDelegates.native_rcl_create_qos_profile_handle(ref qosProfileHandle);
            if (ret != RCLRet.Ok)
            {
                qosProfileHandle.Dispose();
                throw RCLExceptionHelper.CreateFromReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_create_qos_profile_handle)}() failed.");
            }

            return qosProfileHandle;
        }

        internal static void WriteToQosProfileHandle(QosProfile qosProfile, SafeQosProfileHandle qosProfileHandle)
        {
            ToRmwTime(qosProfile.Deadline, out ulong deadlineSec, out ulong deadlineNsec);
            ToRmwTime(qosProfile.Lifespan, out ulong lifespanSec, out ulong lifespanNsec);
            ToRmwTime(qosProfile.LivelinessLeaseDuration, out ulong livelinessLeaseDurationSec, out ulong livelinessLeaseDurationNsec);

            RCLRet ret = RCLdotnetDelegates.native_rcl_write_to_qos_profile_handle(qosProfileHandle,
                history: (int)qosProfile.History,
                depth: qosProfile.Depth,
                reliability: (int)qosProfile.Reliability,
                durability: (int)qosProfile.Durability,
                deadlineSec: deadlineSec,
                deadlineNsec: deadlineNsec,
                lifespanSec: lifespanSec,
                lifespanNsec: lifespanNsec,
                liveliness: (int)qosProfile.Liveliness,
                livelinessLeaseDurationSec: livelinessLeaseDurationSec,
                livelinessLeaseDurationNsec: livelinessLeaseDurationNsec,
                avoidRosNamespaceConventions: qosProfile.AvoidRosNamespaceConventions ? 1 : 0);

            RCLExceptionHelper.CheckReturnValue(ret, $"{nameof(RCLdotnetDelegates.native_rcl_write_to_qos_profile_handle)}() failed.");
        }

        private static void ToRmwTime(TimeSpan timeSpan, out ulong sec, out ulong nsec)
        {
            if (timeSpan == InfiniteDuration)
            {
                // see RMW_DURATION_INFINITE and comment on QosProfile.InfiniteDuration above.
                sec = 9223372036;
                nsec = 854775807;
                return;
            }

            const long NanosecondsPerSecond = 1_000_000_000;
            const long NanosecondsPerTick = NanosecondsPerSecond / TimeSpan.TicksPerSecond;

            long ticks = timeSpan.Ticks;
            long seconds = ticks / TimeSpan.TicksPerSecond;
            long ticksRoundedDownToFullSecond = seconds * TimeSpan.TicksPerSecond;
            long ticksInsideSecond = ticks - ticksRoundedDownToFullSecond;

            sec = (ulong)seconds;
            nsec = (ulong)(ticksInsideSecond * NanosecondsPerTick);
        }

        // This method is intended only for reading from a const rmw_qos_profile_t * - it will perform no memory management on the pointer!
        private static QosProfile ReadConstQoSProfile(QosProfileDelegates.NativeRCLGetConstQosProfileHandleType nativeDelegate)
        {
            IntPtr nativeProfileConst = nativeDelegate();

            return new QosProfile(
                QosProfileDelegates.native_rcl_qos_profile_read_history(nativeProfileConst),
                QosProfileDelegates.native_rcl_qos_profile_read_depth(nativeProfileConst),
                QosProfileDelegates.native_rcl_qos_profile_read_reliability(nativeProfileConst),
                QosProfileDelegates.native_rcl_qos_profile_read_durability(nativeProfileConst),
                new RmwTime(QosProfileDelegates.native_rcl_qos_profile_read_deadline, nativeProfileConst).AsTimespan(),
                new RmwTime(QosProfileDelegates.native_rcl_qos_profile_read_lifespan, nativeProfileConst).AsTimespan(),
                QosProfileDelegates.native_rcl_qos_profile_read_liveliness(nativeProfileConst),
                new RmwTime(QosProfileDelegates.native_rcl_qos_profile_read_liveliness_lease_duration, nativeProfileConst).AsTimespan(),
                QosProfileDelegates.native_rcl_qos_profile_read_avoid_ros_namespace_conventions(nativeProfileConst) != 0);
        }
    }
}
