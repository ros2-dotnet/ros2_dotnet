/* Copyright 2023 Queensland University of Technology
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
    /// Safe handle representing a rcl_clock_t
    /// </summary>
    internal sealed class SafeClockHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        public SafeClockHandle()
            : base(ownsHandle: true)
        {
        }

        protected override bool ReleaseHandle()
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_destroy_clock_handle(handle);
            bool successfullyFreed = ret == RCLRet.Ok;

            Debug.Assert(successfullyFreed);

            return successfullyFreed;
        }
    }
}
