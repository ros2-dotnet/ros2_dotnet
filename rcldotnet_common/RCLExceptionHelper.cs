using System;

namespace ROS2.Common
{
    // TODO: (sh) Check return values to throw the appropirate Exceptions arcording
    // to the Framework Design Guidlines.
    // https://docs.microsoft.com/en-us/dotnet/standard/design-guidelines/exceptions

    // TODO: (sh) Use rcl_get_error_state() for additional information.
    public static class RCLExceptionHelper
    {
        public static void ThrowFromReturnValue(RCLRet value, string messagePrefix = null)
        {
            if (value == RCLRet.Ok)
            {
                throw new ArgumentException($"The return value must not be Ok.", nameof(value));
            }

            var message = messagePrefix == null
                ? value.ToString()
                : messagePrefix + " " + value.ToString();

            throw new Exception(message);
        }

        public static void CheckReturnValue(RCLRet value, string messagePrefix = null)
        {
            if (value == RCLRet.Ok)
            {
                return;
            }

            ThrowFromReturnValue(value, messagePrefix);
        }
    }
}
