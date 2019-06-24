/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using ROS2.Interfaces;

namespace rclcs
{

    public class Subscription<T>: ISubscription<T> 
        where T : IRclcsMessage, new ()
    {
        private rcl_subscription_t handle;
        rcl_node_t nodeHandle;

        private IntPtr subscriptionOptions;
        public rcl_subscription_t Handle { get { return handle; } }

        QualityOfServiceProfile qualityOfServiceProfile;

        internal Action<T> callback;

        private bool disposed;

        public IRclcsMessage CreateMessage()
        {
            return (IRclcsMessage)new T(); ;
        }

        public void TriggerCallback(IRclcsMessage message)
        {
            callback((T)message);
        }

        public Subscription(string topic, Node node, Action<T> callback, QualityOfServiceProfile qualityOfServiceProfile = null)
        {
            this.callback = callback;
            nodeHandle = node.handle;
            handle = NativeMethods.rcl_get_zero_initialized_subscription();
            subscriptionOptions = NativeMethods.rclcs_subscription_create_default_options();

            if (qualityOfServiceProfile == null)
            {
                this.qualityOfServiceProfile = new QualityOfServiceProfile(QosProfiles.DEFAULT);
            }
            else
            {
                this.qualityOfServiceProfile = qualityOfServiceProfile;
            }

            //FIXME(sam): was not able to use a c# struct as qos profile, figure out why and replace the following hack...
            NativeMethods.rclcs_subscription_set_qos_profile(subscriptionOptions, (int)qualityOfServiceProfile.Profile);

            //TODO(sam): Figure out why System.Reflection is not available 
            //when building with colcon/xtool on ubuntu 18.04 and mono 4.5
            //MethodInfo m = typeof(T).GetTypeInfo().GetDeclaredMethod("_GET_TYPE_SUPPORT");
            //IntPtr typeSupportHandle = (IntPtr)m.Invoke(null, new object[] { });

            IRclcsMessage msg = new T();
            IntPtr typeSupportHandle = msg.TypeSupportHandle;
            msg.Dispose();

            Utils.CheckReturnEnum(NativeMethods.rcl_subscription_init(
                                    ref handle,
                                    ref nodeHandle,
                                    typeSupportHandle,
                                    topic,
                                    subscriptionOptions));
        }

        ~Subscription()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
        }

        private void Dispose(bool disposing)
        {
            if(!disposed)
            {
                if(disposing)
                {
                    // Dispose managed resources.
                }

                DestroySubscription();

                disposed = true;
            }
        }


        private void DestroySubscription()
        {
            Utils.CheckReturnEnum(NativeMethods.rcl_subscription_fini(ref handle, ref nodeHandle));
            NativeMethods.rclcs_node_dispose_options(subscriptionOptions);
        }

    }
}
