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
    /// Safe handle representing a rcl_client_t
    /// </summary>
    /// <summary>
    /// Since we need to delete the client handle before the node is released we need to actually hold a
    /// pointer to a rcl_client_t unmanaged structure whose destructor decrements a refCount. Only when
    /// the node refCount is 0 it is deleted. This way, we loose a race in the critical finalization
    /// of the client handle and node handle. This also applies to publisher, subscription and service handles,
    /// which point to a rcl_publisher_t, rcl_subscription_t or rcl_service_t.
    /// </summary>
    internal sealed class SafeClientHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        // Trick with parent handles taken from https://github.com/dotnet/corefx/pull/6366
        // Commit from early 2016, but still in current .NET as of september 2021:
        // https://github.com/dotnet/runtime/blob/57bfe474518ab5b7cfe6bf7424a79ce3af9d6657/src/libraries/Common/src/Interop/Windows/Advapi32/SafeKeyHandle.cs
        //
        // Finding docs about SafeHandle is difficult, but the implementation and examples
        // in github.com/dotnet/runtime (in combination with the discussions in the PRs)
        // should be good enougth. At least are there people that know what they do ;)
        //
        // Needed to change the exact order of statements in ReleaseOrder() as rcl_client_fini()
        // needs the parent handle as well. As far as I understand this there should be no new
        // race conditions.

        private SafeNodeHandle _parent;

        public SafeClientHandle()
            : base(ownsHandle: true)
        {
        }

        protected override bool ReleaseHandle()
        {
            var parent = _parent;
            _parent = null;

            bool successfullyFreed = false;
            if (parent != null)
            {
                Debug.Assert(!parent.IsClosed);
                Debug.Assert(!parent.IsInvalid);

                RCLRet ret = NodeDelegates.native_rcl_destroy_client_handle(handle, parent);
                successfullyFreed = ret == RCLRet.Ok;

                parent.DangerousRelease();
            }

            Debug.Assert(successfullyFreed);

            return successfullyFreed;
        }

        internal void SetParent(SafeNodeHandle parent)
        {
            if (IsInvalid || IsClosed)
            {
                return;
            }

            Debug.Assert(_parent == null);
            Debug.Assert(!parent.IsClosed);
            Debug.Assert(!parent.IsInvalid);

            _parent = parent;

            bool ignored = false;
            _parent.DangerousAddRef(ref ignored);
        }
    }
}
