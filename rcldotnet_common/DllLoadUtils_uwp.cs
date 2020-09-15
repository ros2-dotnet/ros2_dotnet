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

// Based on http://dimitry-i.blogspot.com.es/2013/01/mononet-how-to-dynamically-load-native.html

using System;
using System.Runtime;
using System.Runtime.InteropServices;

namespace ROS2 {
  namespace Utils {
      // This file is nearly identical to DllLoadUtils.cs. however, when compiling for Windows store, the mere presence of some p/invokes causes issues deploying to Hololens
    public class UnsatisfiedLinkError : System.Exception {
      public UnsatisfiedLinkError () : base () { }
      public UnsatisfiedLinkError (string message) : base (message) { }
      public UnsatisfiedLinkError (string message, System.Exception inner) : base (message, inner) { }
    }

    public class DllLoadUtilsFactory {
      public static DllLoadUtils GetDllLoadUtils () {
        return new DllLoadUtilsUWP ();
      }
    }

    public interface DllLoadUtils {
      IntPtr LoadLibrary (string fileName);
      void FreeLibrary (IntPtr handle);
      IntPtr GetProcAddress (IntPtr dllHandle, string name);
    }

    public class DllLoadUtilsUWP : DllLoadUtils {

      [DllImport ("api-ms-win-core-libraryloader-l2-1-0.dll", SetLastError = true, ExactSpelling = true)]
      private static extern IntPtr LoadPackagedLibrary ([MarshalAs (UnmanagedType.LPWStr)] string fileName, int reserved = 0);

      [DllImport ("api-ms-win-core-libraryloader-l1-2-0.dll", SetLastError = true, ExactSpelling = true)]
      private static extern int FreeLibrary (IntPtr handle);

      [DllImport ("api-ms-win-core-libraryloader-l1-2-0.dll", SetLastError = true, ExactSpelling = true)]
      private static extern IntPtr GetProcAddress (IntPtr handle, string procedureName);

      void DllLoadUtils.FreeLibrary (IntPtr handle) {
        FreeLibrary (handle);
      }

      IntPtr DllLoadUtils.GetProcAddress (IntPtr dllHandle, string name) {
        return GetProcAddress (dllHandle, name);
      }

      IntPtr DllLoadUtils.LoadLibrary (string fileName) {
        string libraryName = fileName + "_native.dll";
        IntPtr ptr = LoadPackagedLibrary (libraryName);
        if (ptr == IntPtr.Zero) {
          throw new UnsatisfiedLinkError (libraryName);
        }
        return ptr;
      }
    }
  }
}
