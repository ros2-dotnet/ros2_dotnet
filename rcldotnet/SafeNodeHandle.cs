/* Copyright 2021-2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
 
using System.Diagnostics;
using Microsoft.Win32.SafeHandles;

namespace ROS2
{
    /// <summary>
    /// Safe handle representing a rcl_node_t
    /// </summary>
    /// <summary>
    /// SafePublisherHandle, SafeSubscriptionHandle, SafeServiceHandle and SafeClientHandle
    /// all increase the refCount of their parrent SafeNodeHandles.
    /// This means that the Node does not immediately get released if there is still
    /// one of those alive.
    /// </summary>
    internal sealed class SafeNodeHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        public SafeNodeHandle()
            : base(ownsHandle: true)
        {
        }

        protected override bool ReleaseHandle()
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_destroy_node_handle(handle);
            bool successfullyFreed = ret == RCLRet.Ok;

            Debug.Assert(successfullyFreed);

            return successfullyFreed;
        }
    }
}
