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

using System;
using System.Reflection;

namespace ROS2
{
    internal static class ServiceDefinitionStaticMemberCache<TService, TRequest, TResponse>
        where TService : IRosServiceDefinition<TRequest, TResponse>
        where TRequest : IRosMessage, new()
        where TResponse : IRosMessage, new()
    {
        private static readonly IntPtr s_typeSupport;

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
