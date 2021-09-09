using System;

namespace ROS2
{
    public interface IRosMessage
    {
        // must be implemented on deriving types, gets called via reflection
        // (static abstract interface members are not supported yet.)
        // public static abstract IntPtr __GetTypeSupport();
        // public static abstract SafeHandle __CreateMessageHandle();
        
        void __ReadFromHandle(IntPtr messageHandle);

        void __WriteToHandle(IntPtr messageHandle);
    }
}
