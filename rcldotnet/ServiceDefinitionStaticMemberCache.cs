using System;
using System.Reflection;

namespace ROS2
{
    internal static class ServiceDefinitionStaticMemberCache<TService, TRequest, TResponse>
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IRosMessage, new()
        where TResponse : IRosMessage, new()
    {
        private static IntPtr s_typeSupport;

        static ServiceDefinitionStaticMemberCache()
        {
            TypeInfo typeInfo = typeof(TService).GetTypeInfo();

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
        }

        public static IntPtr GetTypeSupport()
        {
            // mehtod because it could throw.
            if (s_typeSupport == IntPtr.Zero)
            {
                throw new InvalidOperationException($"Type '{typeof(TService).FullName}' did not define a correct __GetTypeSupport mehtod.");
            }
            
            return s_typeSupport;
        }
    }
}
