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

namespace rclcs
{
    public class WaitSet: IDisposable
    {
        private rcl_wait_set_t handle;
        private rcl_allocator_t allocator;
        private bool disposed;

        public WaitSet(IList<ISubscriptionBase> subscriptions = null)
        {
            ulong numberOfSubscriptions;

            if (subscriptions == null)
            {
                numberOfSubscriptions = 0;
                subscriptions = new List<ISubscriptionBase>();
            }
            else
            {
                numberOfSubscriptions = (ulong)subscriptions.Count;
            }

            ulong numberOfGuardConditions = 0;
            ulong numberOfTimers = 0;
            ulong numberOfClients = 0;
            ulong numberOfServices = 0;

            allocator = NativeMethods.rcl_get_default_allocator();
            handle = NativeMethods.rcl_get_zero_initialized_wait_set();

            Utils.CheckReturnEnum(NativeMethods.rcl_wait_set_init(
                ref handle,
                numberOfSubscriptions,
                numberOfGuardConditions,
                numberOfTimers,
                numberOfClients,
                numberOfServices,
                allocator));

            Clear();

            foreach (ISubscriptionBase subscription in subscriptions)
            {
                rcl_subscription_t subscription_handle = subscription.Handle;
                Utils.CheckReturnEnum(NativeMethods.rcl_wait_set_add_subscription(ref handle, ref subscription_handle, UIntPtr.Zero));
            }
        }

        public void Clear()
        {
            Utils.CheckReturnEnum(NativeMethods.rcl_wait_set_clear(ref handle));
        }

        public void Wait(double timeoutSec)
        {
            NativeMethods.rcl_wait(ref handle, Utils.TimeoutSecToNsec(timeoutSec));
        }

        ~WaitSet()
        {
            Dispose();
        }

        public void Dispose()
        {
            if(!disposed)
            {
                Utils.CheckReturnEnum(NativeMethods.rcl_wait_set_fini(ref handle));
                disposed = true;
            }
        }
    }
}
