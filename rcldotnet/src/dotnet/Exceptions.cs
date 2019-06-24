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
    public class RuntimeError : Exception
    {
        public RuntimeError()
        {
        }

        public RuntimeError(string message) : base(message)
        {
        }

        public RuntimeError(string message, Exception inner) : base(message, inner)
        {
        }
    }

    public class NotInitializedException : Exception
    {
        public NotInitializedException()
        {
        }

        public NotInitializedException(string message) : base(message)
        {
        }

        public NotInitializedException(string message, Exception inner) : base(message, inner)
        {
        }
    }

    public class InvalidNodeNameException : Exception
    {
        public InvalidNodeNameException()
        {
        }

        public InvalidNodeNameException(string message) : base(message)
        {
        }

        public InvalidNodeNameException(string message, Exception inner) : base(message, inner)
        {
        }
    }

    public class InvalidNamespaceException : Exception
    {
        public InvalidNamespaceException()
        {
        }

        public InvalidNamespaceException(string message) : base(message)
        {
        }

        public InvalidNamespaceException(string message, Exception inner) : base(message, inner)
        {
        }
    }
}

