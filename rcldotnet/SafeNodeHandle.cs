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
