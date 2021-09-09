using System.Diagnostics;
using Microsoft.Win32.SafeHandles;

namespace ROS2
{
    /// <summary>
    /// Safe handle representing a rmw_request_id_t
    /// </summary>
    internal sealed class SafeRequestIdHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        public SafeRequestIdHandle()
            : base(ownsHandle: true)
        {
        }

        protected override bool ReleaseHandle()
        {
            RCLRet ret = RCLdotnetDelegates.native_rcl_destroy_request_id_handle(handle);
            bool successfullyFreed = ret == RCLRet.Ok;

            Debug.Assert(successfullyFreed);

            return successfullyFreed;
        }
    }
}
