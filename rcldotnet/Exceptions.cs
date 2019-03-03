/* Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
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

namespace ROS2 {


  class NodeNameInvalidIsEmptyStringException : Exception
  {
    public NodeNameInvalidIsEmptyStringException()
    {
    }

    public NodeNameInvalidIsEmptyStringException(string errorString)
        : base(errorString)
    {
    }
  }

  class NodeNameInvalidContainsUnallowedCharactersException : Exception
  {
    public NodeNameInvalidContainsUnallowedCharactersException()
    {
    }

    public NodeNameInvalidContainsUnallowedCharactersException(string errorString)
        : base(errorString)
    {
    }
  }

  class NodeNameInvalidStartsWithNumberException : Exception
  {
    public NodeNameInvalidStartsWithNumberException()
    {
    }

    public NodeNameInvalidStartsWithNumberException(string errorString)
        : base(errorString)
    {
    }
  }

  class NodeNameInvalidTooLongException : Exception
  {
    public NodeNameInvalidTooLongException()
    {
    }

    public NodeNameInvalidTooLongException(string errorString)
        : base(errorString)
    {
    }
  }

  class AlreadyInitException : Exception
  {
    public AlreadyInitException()
    {
    }

    public AlreadyInitException(string errorString)
        : base(errorString)
    {
    }
  }

  class InvalidArgumentException : Exception
  {
    public InvalidArgumentException()
    {
    }

    public InvalidArgumentException(string errorString)
        : base(errorString)
    {
    }
  }

  class BadAllocException : Exception
  {
    public BadAllocException()
    {
    }

    public BadAllocException(string errorString)
        : base(errorString)
    {
    }
  }

  class NodeInvalidNameException : Exception
  {
    public NodeInvalidNameException()
    {
    }

    public NodeInvalidNameException(string errorString)
        : base(errorString)
    {
    }
  }

  class NodeInvalidNamespaceException : Exception
  {
    public NodeInvalidNamespaceException()
    {
    }

    public NodeInvalidNamespaceException(string errorString)
        : base(errorString)
    {
    }
  }

}
