using System;

namespace ROS2
{
    // TODO: (sh) Check return values to throw the appropirate Exceptions arcording
    // to the Framework Design Guidlines.
    // https://docs.microsoft.com/en-us/dotnet/standard/design-guidelines/exceptions

    // TODO: (sh) Use rcl_get_error_state() for additional information.
    internal static class RCLExceptionHelper
    {
        public static Exception CreateFromReturnValue(RCLRet value, string messagePrefix = null)
        {
            if (value == RCLRet.Ok)
            {
                throw new ArgumentException($"The return value must not be Ok.", nameof(value));
            }

            // #define RCUTILS_ERROR_MESSAGE_MAX_LENGTH 1024
            var errorStringBuffer = new byte[1024];
            RCLdotnetDelegates.native_rcl_get_error_string(errorStringBuffer, errorStringBuffer.Length);
            
            string errorString = System.Text.Encoding.UTF8.GetString(errorStringBuffer).TrimEnd('\0');

            RCLdotnetDelegates.native_rcl_reset_error();

            var message = messagePrefix == null
                ? "ReturnCode: " + value.ToString() + " ErrorString: " + errorString
                : messagePrefix + " ReturnCode: " + value.ToString() + " ErrorString: " + errorString;

            return new Exception(message);
        }

        public static void CheckReturnValue(RCLRet value, string messagePrefix = null)
        {
            if (value == RCLRet.Ok)
            {
                return;
            }

            throw CreateFromReturnValue(value, messagePrefix);
        }
    }
}
