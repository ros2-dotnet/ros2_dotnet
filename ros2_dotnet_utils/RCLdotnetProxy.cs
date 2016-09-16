// Copyright 2016 Esteve Fernandez <esteve@apache.org>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Runtime.CompilerServices;
using System.Reflection;

namespace ROS2dotnetUtils {

public class RCLdotnetProxy {
  private static readonly object sync_gettypesupport = new object();
  private static readonly object sync_getrmwid = new object();

  public static string GetTypesupportIdentifier() {
    lock(sync_gettypesupport) {
      Type c = Type.GetType("rcldotnet.RCLdotnet, rcldotnet");
      MethodInfo m = c.GetMethod("GetTypesupportIdentifier");
      return (string)m.Invoke(null, new object[] {});
    }
  }

  public static string GetRMWIdentifier() {
    lock(sync_getrmwid) {
      Type c = Type.GetType("rcldotnet.RCLdotnet, rcldotnet");
      MethodInfo m = c.GetMethod("GetRMWIdentifier");
      return (string)m.Invoke(null, new object[] {});
    }
  }
}
}
