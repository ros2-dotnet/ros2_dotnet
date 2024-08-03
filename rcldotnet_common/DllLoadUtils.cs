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
using System.IO;
using System.Runtime.InteropServices;

namespace ROS2
{
    namespace Utils
    {
        public class UnsatisfiedLinkError : System.Exception
        {
            public UnsatisfiedLinkError() : base() { }
            public UnsatisfiedLinkError(string message) : base(message) { }
            public UnsatisfiedLinkError(string message, System.Exception inner) : base(message, inner) { }
        }

        public class UnknownPlatformError : System.Exception
        {
            public UnknownPlatformError() : base() { }
            public UnknownPlatformError(string message) : base(message) { }
            public UnknownPlatformError(string message, System.Exception inner) : base(message, inner) { }
        }

        public enum Platform
        {
            Unix,
            MacOSX,
            WindowsDesktop,
            UWP,
            Unknown
        }

        public class DllLoadUtilsFactory
        {
            [DllImport("api-ms-win-core-libraryloader-l2-1-0.dll", EntryPoint = "LoadPackagedLibrary", SetLastError = true, ExactSpelling = true)]
            private static extern IntPtr LoadPackagedLibrary([MarshalAs(UnmanagedType.LPWStr)] string fileName, int reserved = 0);

            [DllImport("api-ms-win-core-libraryloader-l1-2-0.dll", EntryPoint = "FreeLibrary", SetLastError = true, ExactSpelling = true)]
            private static extern int FreeLibraryUWP(IntPtr handle);

            [DllImport("kernel32.dll", EntryPoint = "LoadLibraryA", SetLastError = true, ExactSpelling = true)]
            private static extern IntPtr LoadLibraryA(string fileName, int reserved = 0);

            [DllImport("kernel32.dll", EntryPoint = "FreeLibrary", SetLastError = true, ExactSpelling = true)]
            private static extern int FreeLibraryDesktop(IntPtr handle);

            [DllImport("libdl.so.2", EntryPoint = "dlopen")]
            private static extern IntPtr dlopen_unix(string fileName, int flags);

            [DllImport("libdl.so.2", EntryPoint = "dlclose")]
            private static extern int dlclose_unix(IntPtr handle);

            [DllImport("libdl.dylib", EntryPoint = "dlopen")]
            private static extern IntPtr dlopen_macosx(string fileName, int flags);

            [DllImport("libdl.dylib", EntryPoint = "dlclose")]
            private static extern int dlclose_macosx(IntPtr handle);

            private const int RTLD_NOW = 2;

            public static DllLoadUtils GetDllLoadUtils()
            {
                switch (CheckPlatform())
                {
                    case Platform.Unix:
                        return new DllLoadUtilsUnix();
                    case Platform.MacOSX:
                        return new DllLoadUtilsMacOSX();
                    case Platform.WindowsDesktop:
                        return new DllLoadUtilsWindowsDesktop();
                    case Platform.UWP:
                        return new DllLoadUtilsUWP();
                    case Platform.Unknown:
                    default:
                        throw new UnknownPlatformError();
                }
            }

            private static bool IsUWP()
            {
                try
                {
                    IntPtr ptr = LoadPackagedLibrary("api-ms-win-core-libraryloader-l2-1-0.dll");
                    FreeLibraryUWP(ptr);
                    return true;
                }
                catch (TypeLoadException)
                {
                    return false;
                }
            }

            private static bool IsWindowsDesktop()
            {
                try
                {
                    IntPtr ptr = LoadLibraryA("kernel32.dll");
                    FreeLibraryDesktop(ptr);
                    return true;
                }
                catch (TypeLoadException)
                {
                    return false;
                }
            }

            private static bool IsUnix()
            {
                try
                {
                    IntPtr ptr = dlopen_unix("libdl.so.2", RTLD_NOW);
                    dlclose_unix(ptr);
                    return true;
                }
                catch (TypeLoadException)
                {
                    return false;
                }
            }

            private static bool IsMacOSX()
            {
                try
                {
                    IntPtr ptr = dlopen_macosx("libdl.dylib", RTLD_NOW);
                    dlclose_macosx(ptr);
                    return true;
                }
                catch (TypeLoadException)
                {
                    return false;
                }
            }

            private static Platform CheckPlatform()
            {
                if (IsUnix())
                {
                    return Platform.Unix;
                }
                else if (IsMacOSX())
                {
                    return Platform.MacOSX;
                }
                else if (IsWindowsDesktop())
                {
                    return Platform.WindowsDesktop;
                }
                else if (IsUWP())
                {
                    return Platform.UWP;
                }
                else
                {
                    return Platform.Unknown;
                }
            }
        }

        public interface DllLoadUtils
        {
            IntPtr LoadLibrary(string fileName);
            void FreeLibrary(IntPtr handle);
            IntPtr GetProcAddress(IntPtr dllHandle, string name);
            void RegisterNativeFunction<FunctionType>(IntPtr dllHandle, string nativeFunctionName,
                out FunctionType functionDelegate) where FunctionType : Delegate;
        }

        public abstract class DllLoadUtilsAbs : DllLoadUtils
        {
            public abstract IntPtr LoadLibrary(string fileName);

            public abstract void FreeLibrary(IntPtr handle);

            public abstract IntPtr GetProcAddress(IntPtr dllHandle, string name);

            public void RegisterNativeFunction<FunctionType>(IntPtr nativeLibrary, string nativeFunctionName,
                out FunctionType functionDelegate) where FunctionType : Delegate
            {
                IntPtr nativeFunctionPointer = GetProcAddress(nativeLibrary, nativeFunctionName);
                functionDelegate = (FunctionType)Marshal.GetDelegateForFunctionPointer(nativeFunctionPointer, typeof(FunctionType));
            }

            public const string AssemblyDirectory = "ros2";
        }

        public class DllLoadUtilsUWP : DllLoadUtilsAbs
        {

            [DllImport("api-ms-win-core-libraryloader-l2-1-0.dll", SetLastError = true, ExactSpelling = true)]
            private static extern IntPtr LoadPackagedLibraryUWP([MarshalAs(UnmanagedType.LPWStr)] string fileName, int reserved = 0);

            [DllImport("api-ms-win-core-libraryloader-l1-2-0.dll", EntryPoint = "FreeLibrary", SetLastError = true, ExactSpelling = true)]
            private static extern int FreeLibraryUWP(IntPtr handle);

            [DllImport("api-ms-win-core-libraryloader-l1-2-0.dll", EntryPoint = "GetProcAddress", SetLastError = true, ExactSpelling = true)]
            private static extern IntPtr GetProcAddressUWP(IntPtr handle, string procedureName);

            public override void FreeLibrary(IntPtr handle) => FreeLibraryUWP(handle);

            public override IntPtr GetProcAddress(IntPtr dllHandle, string name) => GetProcAddressUWP(dllHandle, name);

            public override IntPtr LoadLibrary(string fileName)
            {
                string libraryName = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, AssemblyDirectory, fileName + "_native.dll");
                IntPtr ptr = LoadPackagedLibraryUWP(libraryName);
                if (ptr == IntPtr.Zero)
                {
                    throw new UnsatisfiedLinkError(libraryName);
                }
                return ptr;
            }
        }

        public class DllLoadUtilsWindowsDesktop : DllLoadUtilsAbs
        {

            [DllImport("kernel32.dll", EntryPoint = "GetLastError", SetLastError = true, ExactSpelling = true)]
            private static extern int GetLastError();

            [DllImport("kernel32.dll", EntryPoint = "LoadLibraryA", SetLastError = true, ExactSpelling = true)]
            private static extern IntPtr LoadLibraryA(string fileName, int reserved = 0);

            [DllImport("kernel32.dll", EntryPoint = "FreeLibrary", SetLastError = true, ExactSpelling = true)]
            private static extern int FreeLibraryWin(IntPtr handle);

            [DllImport("kernel32.dll", EntryPoint = "GetProcAddress", SetLastError = true, ExactSpelling = true)]
            private static extern IntPtr GetProcAddressWin(IntPtr handle, string procedureName);

            public override void FreeLibrary(IntPtr handle) => FreeLibraryWin(handle);

            public override IntPtr GetProcAddress(IntPtr dllHandle, string name) => GetProcAddressWin(dllHandle, name);

            public override IntPtr LoadLibrary(string fileName)
            {
                string libraryName = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, AssemblyDirectory, fileName + "_native.dll");
                IntPtr ptr = LoadLibraryA(libraryName);
                if (ptr == IntPtr.Zero)
                {
                    throw new UnsatisfiedLinkError(libraryName);
                }
                return ptr;
            }
        }

        internal class DllLoadUtilsUnix : DllLoadUtilsAbs
        {

            [DllImport("libdl.so.2", ExactSpelling = true)]
            private static extern IntPtr dlopen(string fileName, int flags);

            [DllImport("libdl.so.2", ExactSpelling = true)]
            private static extern IntPtr dlsym(IntPtr handle, string symbol);

            [DllImport("libdl.so.2", ExactSpelling = true)]
            private static extern int dlclose(IntPtr handle);

            [DllImport("libdl.so.2", ExactSpelling = true)]
            private static extern IntPtr dlerror();

            private const int RTLD_NOW = 2;

            public override void FreeLibrary(IntPtr handle) => dlclose(handle);

            public override IntPtr GetProcAddress(IntPtr dllHandle, string name)
            {
                // clear previous errors if any
                dlerror();
                var res = dlsym(dllHandle, name);
                var errPtr = dlerror();
                if (errPtr != IntPtr.Zero)
                {
                    throw new Exception("dlsym: " + Marshal.PtrToStringAnsi(errPtr));
                }
                return res;
            }

            public override IntPtr LoadLibrary(string fileName)
            {
                string libraryName = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, AssemblyDirectory, "lib" + fileName + "_native.so");
                IntPtr ptr = dlopen(libraryName, RTLD_NOW);
                if (ptr == IntPtr.Zero)
                {
                    throw new UnsatisfiedLinkError(libraryName);
                }
                return ptr;
            }
        }

        internal class DllLoadUtilsMacOSX : DllLoadUtilsAbs
        {

            [DllImport("libdl.dylib", ExactSpelling = true)]
            private static extern IntPtr dlopen(string fileName, int flags);

            [DllImport("libdl.dylib", ExactSpelling = true)]
            private static extern IntPtr dlsym(IntPtr handle, string symbol);

            [DllImport("libdl.dylib", ExactSpelling = true)]
            private static extern int dlclose(IntPtr handle);

            [DllImport("libdl.dylib", ExactSpelling = true)]
            private static extern IntPtr dlerror();

            private const int RTLD_NOW = 2;

            public override void FreeLibrary(IntPtr handle)
            {
                dlclose(handle);
            }

            public override IntPtr GetProcAddress(IntPtr dllHandle, string name)
            {
                // clear previous errors if any
                dlerror();
                var res = dlsym(dllHandle, name);
                var errPtr = dlerror();
                if (errPtr != IntPtr.Zero)
                {
                    throw new Exception("dlsym: " + Marshal.PtrToStringAnsi(errPtr));
                }
                return res;
            }

            public override IntPtr LoadLibrary(string fileName)
            {
                string libraryName = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, AssemblyDirectory, "lib" + fileName + "_native.dylib");
                IntPtr ptr = dlopen(libraryName, RTLD_NOW);
                if (ptr == IntPtr.Zero)
                {
                    throw new UnsatisfiedLinkError(libraryName);
                }
                return ptr;
            }
        }
    }
}
