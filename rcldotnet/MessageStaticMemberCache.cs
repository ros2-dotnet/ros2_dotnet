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
using System.Runtime.InteropServices;

namespace ROS2
{
    internal static class MessageStaticMemberCache<T>
        where T : IRosMessage
    {
        private static readonly IntPtr s_typeSupport;
        private static readonly Func<SafeHandle> s_createMessageHandle;

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
            // method because it could throw.
            if (s_typeSupport == IntPtr.Zero)
            {
                throw new InvalidOperationException($"Type '{typeof(T).FullName}' did not define a correct __GetTypeSupport method.");
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
                throw new InvalidOperationException($"Type '{typeof(T).FullName}' did not define a correct __CreateMessageHandle method.");
            }
        }
    }
}
