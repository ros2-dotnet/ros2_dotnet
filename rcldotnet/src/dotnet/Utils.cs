/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;

namespace rclcs
{
    public static class Utils
    {
        internal static void CheckReturnEnum(int ret)
        {
            string errorMessage = Utils.PopRclErrorString();

            switch ((RCLReturnEnum)ret)
            {
                case RCLReturnEnum.RCL_RET_OK:
                    break;
                case RCLReturnEnum.RCL_RET_NODE_INVALID_NAME:
                    throw new InvalidNodeNameException(errorMessage);
                case RCLReturnEnum.RCL_RET_NODE_INVALID_NAMESPACE:
                    throw new InvalidNamespaceException(errorMessage);
                default:
                    throw new RuntimeError(errorMessage);
            }
        }

        public static string GetRclErrorString()
        {
            IntPtr errorStringPtr = NativeMethods.rclcs_get_error_string();
            string errorString = MarshallingHelpers.PtrToString(errorStringPtr);
            NativeMethods.rclcs_dispose_error_string(errorStringPtr);
            return errorString;
        }

        public static string PopRclErrorString()
        {
            string errorString = GetRclErrorString();
            NativeMethods.rcl_reset_error();
            return errorString;
        }

        public static ulong TimeoutSecToNsec(double timeoutSec)
        {
            if(timeoutSec < 0)
            {
                throw new RuntimeError("Negative timeouts are not allowed, timeout was " + timeoutSec + " seconds.");
            }
            return (ulong)(timeoutSec * Constants.S_TO_NS);
        }
    }
}
