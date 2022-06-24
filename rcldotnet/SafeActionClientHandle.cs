using System.Diagnostics;
using Microsoft.Win32.SafeHandles;

namespace ROS2
{
    /// <summary>
    /// Safe handle representing a rcl_action_client_t
    /// </summary>
    /// <summary>
    /// Since we need to delete the action client handle before the node is released we need to actually hold a
    /// pointer to a rcl_action_client_t unmanaged structure whose destructor decrements a refCount. Only when
    /// the node refCount is 0 it is deleted. This way, we loose a race in the critical finalization
    /// of the client handle and node handle.
    /// </summary>
    internal sealed class SafeActionClientHandle : SafeHandleZeroOrMinusOneIsInvalid
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

        public SafeActionClientHandle()
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

                RCLRet ret = NodeDelegates.native_rcl_action_destroy_client_handle(handle, parent);
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
