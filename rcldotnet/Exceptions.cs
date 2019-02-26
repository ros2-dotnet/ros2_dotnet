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

  class InvalidNodeNameException : Exception
  {

    string rmw_error_string_;

    public InvalidNodeNameException()
    {
    }

    public InvalidNodeNameException(string name, string rmw_error_string)
        : base(String.Format("Invalid Node Name: {0}", name))
    {
      rmw_error_string_ = rmw_error_string;
    }

    public string GetRMWErrorString()
    {
      return rmw_error_string_;
    }
  }
}
