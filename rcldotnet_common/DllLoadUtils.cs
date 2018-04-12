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

// Based on http://dimitry-i.blogspot.com.es/2013/01/mononet-how-to-dynamically-load-native.html

using System;
using System.Runtime;
using System.Runtime.InteropServices;

namespace ROS2 {
  namespace Utils {
    public class UnsatisfiedLinkError : System.Exception {
      public UnsatisfiedLinkError () : base () { }
      public UnsatisfiedLinkError (string message) : base (message) { }
      public UnsatisfiedLinkError (string message, System.Exception inner) : base (message, inner) { }
    }

    public class DllLoadUtilsFactory {
      public static DllLoadUtils GetDllLoadUtils () {
        DllLoadUtils dllLoadUtils = IsUnix () ? (DllLoadUtils) new DllLoadUtilsUnix () : new DllLoadUtilsWindows ();
        return dllLoadUtils;
      }

      private static bool IsUnix () {
        // See http://www.mono-project.com/docs/faq/technical/#how-to-detect-the-execution-platform
        var p = (int) Environment.OSVersion.Platform;
        return (p == 4) || (p == 6) || (p == 128);
      }
    }

    public interface DllLoadUtils {
      IntPtr LoadLibrary (string fileName);
      void FreeLibrary (IntPtr handle);
      IntPtr GetProcAddress (IntPtr dllHandle, string name);
    }

    public class DllLoadUtilsWindows : DllLoadUtils {
      void DllLoadUtils.FreeLibrary (IntPtr handle) {
        FreeLibrary (handle);
      }

      IntPtr DllLoadUtils.GetProcAddress (IntPtr dllHandle, string name) {
        return GetProcAddress (dllHandle, name);
      }

      IntPtr DllLoadUtils.LoadLibrary (string fileName) {
        string libraryName = fileName + "_native.dll";
        IntPtr ptr = LoadLibrary (libraryName);
        if (ptr == IntPtr.Zero) {
          throw new UnsatisfiedLinkError (libraryName);
        }
        return ptr;
      }

      [DllImport ("kernel32.dll")]
      private static extern IntPtr LoadLibrary (string fileName);

      [DllImport ("kernel32.dll")]
      private static extern int FreeLibrary (IntPtr handle);

      [DllImport ("kernel32.dll")]
      private static extern IntPtr GetProcAddress (IntPtr handle, string procedureName);
    }

    internal class DllLoadUtilsUnix : DllLoadUtils {
      public IntPtr LoadLibrary (string fileName) {
        string libraryName = "lib" + fileName + "_native.so";
        Console.WriteLine ("Loading library: " + libraryName);
        IntPtr ptr = dlopen (libraryName, RTLD_NOW);
        if (ptr == IntPtr.Zero) {
          throw new UnsatisfiedLinkError (libraryName);
        }
        return ptr;
      }

      public void FreeLibrary (IntPtr handle) {
        dlclose (handle);
      }

      public IntPtr GetProcAddress (IntPtr dllHandle, string name) {
        // clear previous errors if any
        dlerror ();
        var res = dlsym (dllHandle, name);
        var errPtr = dlerror ();
        if (errPtr != IntPtr.Zero) {
          throw new Exception ("dlsym: " + Marshal.PtrToStringAnsi (errPtr));
        }
        return res;
      }

      const int RTLD_NOW = 2;

      [DllImport ("libdl.so")]
      private static extern IntPtr dlopen (String fileName, int flags);

      [DllImport ("libdl.so")]
      private static extern IntPtr dlsym (IntPtr handle, String symbol);

      [DllImport ("libdl.so")]
      private static extern int dlclose (IntPtr handle);

      [DllImport ("libdl.so")]
      private static extern IntPtr dlerror ();
    }
  }
}