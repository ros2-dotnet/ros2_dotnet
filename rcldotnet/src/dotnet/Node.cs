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
using System.Collections.Generic;
using ROS2.Interfaces;

namespace rclcs
{
    /// <summary>
    /// Represents a managed ROS node
    /// </summary>
    public class Node: INode
    {
        internal rcl_node_t handle;

        private IntPtr defaultNodeOptions;

        private bool disposed;

        private IList<ISubscriptionBase> subscriptions;
        private IList<IPublisherBase> publishers;

        public IList<ISubscriptionBase> Subscriptions { get { return subscriptions; } }

        public Node(string nodeName, Context context, string nodeNamespace = null)
        {
            subscriptions = new List<ISubscriptionBase>();
            publishers = new List<IPublisherBase>();

            if (nodeNamespace == null) { nodeNamespace = "/";  }
            if (context.Ok)
            {
                handle = NativeMethods.rcl_get_zero_initialized_node();
                defaultNodeOptions = NativeMethods.rclcs_node_create_default_options();

                Utils.CheckReturnEnum(NativeMethods.rcl_node_init(ref handle, nodeName, nodeNamespace, ref context.handle, defaultNodeOptions));

            }
            else
            {
                throw new NotInitializedException();
            }
        }

        public string Name
        {
            get { return MarshallingHelpers.PtrToString(NativeMethods.rcl_node_get_name(ref handle)); }
        }

        public string Namespace
        {
            get { return MarshallingHelpers.PtrToString(NativeMethods.rcl_node_get_namespace(ref handle)); }
        }

        ~Node()
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

                foreach(ISubscriptionBase subscription in subscriptions)
                {
                    subscription.Dispose();
                }

                foreach(IPublisherBase publisher in publishers)
                {
                    publisher.Dispose();
                }

                DestroyNode();

                disposed = true;
            }
        }

        public void DestroyNode()
        {
            Utils.CheckReturnEnum(NativeMethods.rcl_node_fini(ref handle));
            NativeMethods.rclcs_node_dispose_options(defaultNodeOptions);
        }
       

        public Publisher<T> CreatePublisher<T>(string topic) where T : IRclcsMessage, new()
        {
            Publisher<T> publisher = new Publisher<T>(topic, this);
            publishers.Add(publisher);
            return publisher;
        }

        public Subscription<T> CreateSubscription<T>(string topic, Action<T> callback, QualityOfServiceProfile qos = null) where T : IRclcsMessage, new ()
        {
            if(qos == null)
            {
                qos = new QualityOfServiceProfile(QosProfiles.DEFAULT);
            }

            Subscription<T> subscription = new Subscription<T>(topic, this, callback, qos);
            subscriptions.Add(subscription);
            return subscription;
        }

    }


}

