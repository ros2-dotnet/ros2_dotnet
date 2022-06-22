/* Copyright 2022 Stefan Hoffmann <stefan.hoffmann@schiller.de>
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
    internal static class ActionDefinitionStaticMemberCache<TAction, TGoal, TResult, TFeedback>
        where TAction : IRosActionDefinition<TGoal, TResult, TFeedback>
        where TGoal : IRosMessage, new()
        where TResult : IRosMessage, new()
        where TFeedback : IRosMessage, new()
    {
        private static readonly IntPtr s_typeSupport;
        private static readonly Func<IRosActionSendGoalRequest<TGoal>> s_createSendGoalRequest;
        private static readonly Func<SafeHandle> s_createSendGoalRequestHandle;
        private static readonly Func<IRosActionSendGoalResponse> s_createSendGoalResponse;
        private static readonly Func<SafeHandle> s_createSendGoalResponseHandle;
        private static readonly Func<IRosActionGetResultRequest> s_createGetResultRequest;
        private static readonly Func<SafeHandle> s_createGetResultRequestHandle;
        private static readonly Func<IRosActionGetResultResponse<TResult>> s_createGetResultResponse;
        private static readonly Func<SafeHandle> s_createGetResultResponseHandle;
        private static readonly Func<IRosActionFeedbackMessage<TFeedback>> s_createFeedbackMessage;
        private static readonly Func<SafeHandle> s_createFeedbackMessageHandle;

        static ActionDefinitionStaticMemberCache()
        {
            TypeInfo typeInfo = typeof(TAction).GetTypeInfo();

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

            s_createSendGoalRequest = CreateDelegateForDeclaredMethod<Func<IRosActionSendGoalRequest<TGoal>>>(
                typeInfo,
                "__CreateSendGoalRequest");

            s_createSendGoalRequestHandle = CreateDelegateForDeclaredMethod<Func<SafeHandle>>(
                typeInfo,
                "__CreateSendGoalRequestHandle");

            s_createSendGoalResponse = CreateDelegateForDeclaredMethod<Func<IRosActionSendGoalResponse>>(
                typeInfo,
                "__CreateSendGoalResponse");

            s_createSendGoalResponseHandle = CreateDelegateForDeclaredMethod<Func<SafeHandle>>(
                typeInfo,
                "__CreateSendGoalResponseHandle");

            s_createGetResultRequest = CreateDelegateForDeclaredMethod<Func<IRosActionGetResultRequest>>(
                typeInfo,
                "__CreateGetResultRequest");

            s_createGetResultRequestHandle = CreateDelegateForDeclaredMethod<Func<SafeHandle>>(
                typeInfo,
                "__CreateGetResultRequestHandle");

            s_createGetResultResponse = CreateDelegateForDeclaredMethod<Func<IRosActionGetResultResponse<TResult>>>(
                typeInfo,
                "__CreateGetResultResponse");

            s_createGetResultResponseHandle = CreateDelegateForDeclaredMethod<Func<SafeHandle>>(
                typeInfo,
                "__CreateGetResultResponseHandle");

            s_createFeedbackMessage = CreateDelegateForDeclaredMethod<Func<IRosActionFeedbackMessage<TFeedback>>>(
                typeInfo,
                "__CreateFeedbackMessage");

            s_createFeedbackMessageHandle = CreateDelegateForDeclaredMethod<Func<SafeHandle>>(
                typeInfo,
                "__CreateFeedbackMessageHandle");
        }

        public static IntPtr GetTypeSupport()
        {
            // This is a Method because it could throw.
            if (s_typeSupport == IntPtr.Zero)
            {
                throw CreateMethodNotDefinedCorrectlyException("__GetTypeSupport");
            }

            return s_typeSupport;
        }

        public static IRosActionSendGoalRequest<TGoal> CreateSendGoalRequest()
        {
            if (s_createSendGoalRequest != null)
            {
                return s_createSendGoalRequest();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateSendGoalRequest");
            }
        }

        public static SafeHandle CreateSendGoalRequestHandle()
        {
            if (s_createSendGoalRequestHandle != null)
            {
                return s_createSendGoalRequestHandle();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateSendGoalRequestHandle");
            }
        }

        public static IRosActionSendGoalResponse CreateSendGoalResponse()
        {
            if (s_createSendGoalResponse != null)
            {
                return s_createSendGoalResponse();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateSendGoalResponse");
            }
        }

        public static SafeHandle CreateSendGoalResponseHandle()
        {
            if (s_createSendGoalResponseHandle != null)
            {
                return s_createSendGoalResponseHandle();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateSendGoalResponseHandle");
            }
        }

        public static IRosActionGetResultRequest CreateGetResultRequest()
        {
            if (s_createGetResultRequest != null)
            {
                return s_createGetResultRequest();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateGetResultRequest");
            }
        }

        public static SafeHandle CreateGetResultRequestHandle()
        {
            if (s_createGetResultRequestHandle != null)
            {
                return s_createGetResultRequestHandle();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateGetResultRequestHandle");
            }
        }

        public static IRosActionGetResultResponse<TResult> CreateGetResultResponse()
        {
            if (s_createGetResultResponse != null)
            {
                return s_createGetResultResponse();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateGetResultResponse");
            }
        }

        public static SafeHandle CreateGetResultResponseHandle()
        {
            if (s_createGetResultResponseHandle != null)
            {
                return s_createGetResultResponseHandle();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateGetResultResponseHandle");
            }
        }

        public static IRosActionFeedbackMessage<TFeedback> CreateFeedbackMessage()
        {
            if (s_createFeedbackMessage != null)
            {
                return s_createFeedbackMessage();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateFeedbackMessage");
            }
        }

        public static SafeHandle CreateFeedbackMessageHandle()
        {
            if (s_createFeedbackMessageHandle != null)
            {
                return s_createFeedbackMessageHandle();
            }
            else
            {
                throw CreateMethodNotDefinedCorrectlyException("__CreateFeedbackMessageHandle");
            }
        }

        private static TDelegate CreateDelegateForDeclaredMethod<TDelegate>(TypeInfo typeInfo, string methodName)
            where TDelegate : Delegate
        {
            MethodInfo methodInfo = typeInfo.GetDeclaredMethod(methodName);
            if (methodInfo != null)
            {
                try
                {
                    return (TDelegate)methodInfo.CreateDelegate(typeof(TDelegate));
                }
                catch
                {
                    return null;
                }
            }
            else
            {
                return null;
            }
        }

        private static Exception CreateMethodNotDefinedCorrectlyException(string methodName)
        {
            return new InvalidOperationException($"Type '{typeof(TAction).FullName}' did not define a correct {methodName} method.");
        }
    }
}
