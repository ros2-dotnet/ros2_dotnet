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
    public static class Rclcs
    {
        // NOTE(sam): Init context when created instaid?

        public static void Init(Context context)
        {
            context.Init();
        }

        public static void Shutdown(Context context)
        {
            context.Shutdown();
        }

        public static Node CreateNode(string nodeName, Context context, string nodeNamespace = null)
        {
            return new Node(nodeName, context, nodeNamespace: nodeNamespace);
        }

        public static void Spin(INode node, Context context)
        {
            while (Ok(context))
            {
                SpinOnce(node, 0.1);
            }
        }

        public static bool Ok(Context context)
        {
            return NativeMethods.rcl_context_is_valid(ref context.handle);
        }

        public static void SpinOnce(INode node, double timeoutSec)
        {
            // NOTE(sam): Only single thread, add support for other executors?
            if (timeoutSec > 0.0001d)
            {
                WaitSet waitSet = new WaitSet(node.Subscriptions);
                waitSet.Wait(timeoutSec);
            }

            foreach(ISubscriptionBase subscription in node.Subscriptions)
            {
                IRclcsMessage message = subscription.CreateMessage();
                bool gotMessage = Take(subscription, message);

                if (gotMessage)
                {
                    subscription.TriggerCallback(message);
                }
            }

        }

        public static bool Take(ISubscriptionBase subscription, IRclcsMessage message)
        {
            rcl_subscription_t subscription_handle = subscription.Handle;
            IntPtr message_handle = message.Handle;
            RCLReturnEnum ret = (RCLReturnEnum)NativeMethods.rcl_take(ref subscription_handle, message_handle, IntPtr.Zero);
            return ret == RCLReturnEnum.RCL_RET_OK;
        }
    }
}
