ROS2 for .NET
=============

Build status
------------

| Target | Status |
|----------|--------|
| **Universal Windows Platform (x86/x64)** | ![Build (UWP)](https://github.com/ros2-dotnet/ros2_dotnet/workflows/Build%20(UWP)/badge.svg) |
| **Windows Desktop (x64)** | ![Build (Desktop)](https://github.com/ros2-dotnet/ros2_dotnet/workflows/Build%20(Desktop)/badge.svg) |
| **Linux** | ![Build (Desktop)](https://github.com/ros2-dotnet/ros2_dotnet/workflows/Build%20(Linux)/badge.svg) |

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for .NET Core and .NET Standard.

Features
--------

The current set of features include:
- Generation of all builtin ROS types
- Support for publishers and subscriptions
- Cross-platform support (Linux, Windows, Windows IoT Core, UWP)

What's missing?
---------------

Lots of things!
- Unicode types
- String constants (specifically BoundedString)
- Nested types
- Component nodes
- Clients and services
- Actions
- Tests
- Documentation
- More examples (e.g. IoT, VB, UWP, HoloLens, etc.)

Sounds great, how can I try this out?
-------------------------------------

First of all install the standard ROS2 dependencies for your operating system
of choice (**NOTE**: only do this if building Windows Desktop or Linux. For UWP,
see the relevant section below): https://github.com/ros2/ros2/wiki/Installation#building-from-source

Next make sure you've either installed .Net Core (preferred)
https://www.microsoft.com/net/learn/get-started or Mono
https://www.mono-project.com/. (**NOTE**: For building unit tests, .NET 6 is
required).

For running on Linux or Windows Desktop, one can build `ros2_dotnet` (along with
all desired packages containing interface definitions) as an overlay on top
of an existing ROS2 installation. The `ros2_dotnet_foxy.repos` contains all
necessary repositories to build the core `ros2_dotnet` project along with all
standard ROS2 interface packages. If you are using other packages which provide
interface definitions, those must also be included in the `ros2_dotnet` workspace
in order for .NET bindings to be generated. (NOTE: if you wish to build the
core of ROS2 from source, everything through the `rcl` layer is required.)

For running within UWP (Universal Windows Platform) applications, the entire
core of ROS2 must be compiled for UWP compatibility.

Windows (Desktop)
-----------------
Assuming you've installed ROS2 (pre-built binary packages) to the directory
c:\dev\ros2_foxy per the official [installation instructions](https://index.ros.org/doc/ros2/Installation/Foxy/Windows-Install-Binary/),
run the following from an Administrator Visual Studio 2019 Developer Command
Prompt:

(**NOTE**: Building as an overlay on top of *binary* distributions of ROS2 has
presented some challenges. As of this writing, you may also need to include the
`rosidl` package due to some unicode/locale compatibility problems.
This is done for you below in the line preceding `colcon build`. This step
can/should be omitted if building on top of a built-from-source ROS2 workspace)

```
call \dev\ros2_foxy\local_setup.bat
md \dev\ros2_dotnet_ws\src
cd \dev\ros2_dotnet_ws
curl -sk https://raw.githubusercontent.com/ros2-dotnet/ros2_dotnet/master/ros2_dotnet_foxy.repos -o ros2_dotnet_foxy.repos
vcs import \dev\ros2_dotnet_ws\src < ros2_dotnet_foxy.repos
git clone --branch foxy https://github.com/ros2/rosidl src\ros2\rosidl
colcon build --merge-install
```


Linux
-----
Assuming ROS2 foxy installed to the standard location, run the following commands:
```
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/ros2-dotnet/ros2_dotnet/master/ros2_dotnet_foxy.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet_foxy.repos
colcon build
```

Universal Windows Platform (Win32, Win64)
----------------------------------------------
We'll build this in two steps, first `ament` (the build system) and related tools
which will run natively on the host, followed by ROS2 itself, built for UWP and
the target architecture.

If you have previously installed ROS2 dependencies (OpenSSL, tinyxml, log4cxx,
etc) it is strongly recommended to uninstall those dependencies before building
to avoid any non-UWP binaries getting pulled into the build.

ament
-----

```
md \dev\ament\src
cd \dev\ament
curl -sk https://raw.githubusercontent.com/ros2-dotnet/ros2_dotnet/master/ament_dotnet_uwp.repos -o ament_dotnet_uwp.repos
vcs import src < ament_dotnet_uwp.repos
colcon build --merge-install
call install\local_setup.bat
```

UWP
---

Replace `%TARGET_ARCH%` with Win32 or x64

```
md \dev\ros2\src
cd \dev\ros2
curl -sk https://raw.githubusercontent.com/ros2-dotnet/ros2_dotnet/master/ros2_dotnet_uwp.repos -o ros2_dotnet_uwp.repos
vcs import src < ros2_dotnet_uwp.repos
cd \dev\ament
call install\local_setup.bat
cd \dev\ros2
colcon build --merge-install --packages-ignore rmw_fastrtps_dynamic_cpp rcl_logging_log4cxx rcl_logging_spdlog ros2trace tracetools_launch tracetools_read tracetools_test tracetools_trace --cmake-args -A "%TARGET_ARCH%" -DCMAKE_SYSTEM_NAME=WindowsStore -DCMAKE_SYSTEM_VERSION=10.0.17763 -DTHIRDPARTY=ON -DINSTALL_EXAMPLES=OFF -DBUILD_TESTING=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop
```

Now you can just run a bunch of examples.

### Publisher and subscriber

Publisher:

Windows
-------

```
call \dev\ros2_dotnet_ws\install\local_setup.bat

ros2 run rcldotnet_examples rcldotnet_talker
```

Linux
-----

```
. ~/ros2_dotnet_ws/install/local_setup.sh

ros2 run rcldotnet_examples rcldotnet_talker
```

Subscriber:

Windows
-------

```
call \dev\ros2_dotnet_ws\install\local_setup.bat

ros2 run rcldotnet_examples rcldotnet_listener
```

Linux
-----

```
. ~/ros2_dotnet_ws/install/local_setup.sh

ros2 run rcldotnet_examples rcldotnet_listener
```

Enjoy!

## Using generated DLLs in your UWP application from Visual Studio

Create a new Visual Studio project (Visual C# - Windows Universal - Empty app).

In Solution Explorer panel:
```
right click on Universal Windows project - Add - Existing item...
```
and include every DLL file from `{your_ros2_uwp_ws}\install\bin`. Now select all of these files in Solution Explorer and check/set the properties:
```
Build action: Content
Copy to output directory: Copy always
```
This allows you to retrieve the files in the same directory as the assembly.

Next step, in Solution Explorer panel:
```
right click on References - Add reference...
```
and include `{your_ros2_uwp_ws}\install\lib\rcldotnet\dotnet\rcldotnet_assemblies.dll`, `{your_ros2_uwp_ws}\install\lib\rcldotnet\dotnet\rcldotnet_common.dll` and `{your_ros2_uwp_ws}\install\lib\std_msgs\dotnet\std_msgs_assemblies.dll`.

Now you can include your ROS2 code in MainPage.xaml.cs script, compile your project and run it on HoloLens Emulator or HoloLens physical device.

## Using generated DLLs in your UWP application from Unity
**NOTE: _Tested on `Unity 2018.2.8f1`_**

Create a new Unity project and set up the following editor properties.

### Build Settings
Set your target platform properly:
```
File - Build Settings - Universal Windows Platform - Switch Platform
```
then set UWP build settings:
```
Target Device: HoloLens
Build Type: D3D
SDK: Latest installed
Visual Studio Version: Latest installed
Build and Run on: Local Machine and Windows Phone
Build Configuration: Release
```
and let unchecked the rest.

### Player Settings

- Other Settings

  **Configuration**
  ```
  Scripting Runtime Version: .NET 4.x Equivalent
  Scripting Backend: .NET
  Api Compatibility Level: .NET 4.x
  ```

- Publishing Settings

  **Capabilities**
  - [x] InternetClient
  - [x] InternetClientServer
  - [x] PrivateNetworkClientServer

- XR Settings
  - [x] Virtual Reality Supported

  Virtual Reality SDKs
    > Windows Mixed Reality

### Add files
Create Assets/Plugins and Assets/Scripts folders.

In Plugins folder include every DLL file from `{your_ros2_uwp_ws}\install\bin`. Also include `{your_ros2_uwp_ws}\install\lib\rcldotnet\dotnet\rcldotnet_assemblies.dll`, `{your_ros2_uwp_ws}\install\lib\rcldotnet\dotnet\rcldotnet_common.dll` and `{your_ros2_uwp_ws}\install\lib\std_msgs\dotnet\std_msgs_assemblies.dll`.

In Scripts folder create your C# scripts and attach them to a scene GameObject to execute them when the app starts.

Finally, build your project to generate a Visual Studio solution.
```
File - Build Settings - Build
```

### Unity generated Visual Studio solution
Open the VS solution generated after building your Unity project.
In Solution Explorer panel:
```
right click on Universal Windows project - Add - Existing item...
```
and include every DLL file from `{your_ros2_uwp_ws}\install\bin`. Now select all of these files in Solution Explorer and check/set the properties:
```
Build action: Content
Copy to output directory: Copy always
```
This allows you to retrieve the files in the same directory as the assembly.

Next step, in Solution Explorer panel:
```
right click on References - Add reference...
```
and include `{your_ros2_uwp_ws}\install\lib\rcldotnet\dotnet\rcldotnet_assemblies.dll`, `{your_ros2_uwp_ws}\install\lib\rcldotnet\dotnet\rcldotnet_common.dll` and `{your_ros2_uwp_ws}\install\lib\std_msgs\dotnet\std_msgs_assemblies.dll`.

Finally compile your project for UWP (32 bits) and run it on HoloLens Emulator or HoloLens physical device.
