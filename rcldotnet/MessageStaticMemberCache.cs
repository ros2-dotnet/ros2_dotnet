using System;
using System.Reflection;
using System.Runtime.InteropServices;

namespace ROS2
{
    internal static class MessageStaticMemberCache<T>
        where T : IRosMessage
    {
        private static IntPtr s_typeSupport;
        private static Func<SafeHandle> s_createMessageHandle;

        static MessageStaticMemberCache()
        {
            TypeInfo typeInfo = typeof(T).GetTypeInfo();

            MethodInfo getTypeSupport = typeInfo.GetDeclaredMethod("__GetTypeSupport");
            if (getTypeSupport != null)
            {
                try
                {
                    s_typeSupport = (IntPtr)getTypeSupport.Invoke(null, new object[] { });
                }
                catch
                {
                    s_typeSupport = IntPtr.Zero;
                }
            }
            else
            {
                s_typeSupport = IntPtr.Zero;
            }

            MethodInfo createMessageHandle = typeInfo.GetDeclaredMethod("__CreateMessageHandle");
            if (createMessageHandle != null)
            {
                try
                {
                    s_createMessageHandle = (Func<SafeHandle>)createMessageHandle.CreateDelegate(typeof(Func<SafeHandle>));
                }
                catch
                {
                    s_createMessageHandle = null;
                }
            }
            else
            {
                s_createMessageHandle = null;
            }
        }

        public static IntPtr GetTypeSupport()
        {
            // mehtod because it could throw.
            if (s_typeSupport == IntPtr.Zero)
            {
                throw new InvalidOperationException($"Type '{typeof(T).FullName}' did not define a correct __GetTypeSupport mehtod.");
            }
            
            return s_typeSupport;
        }

        public static SafeHandle CreateMessageHandle()
        {
            if (s_createMessageHandle != null)
            {
                return s_createMessageHandle();
            }
            else
            {
                throw new InvalidOperationException($"Type '{typeof(T).FullName}' did not define a correct __CreateMessageHandle mehtod.");
            }
        }
    }
}
