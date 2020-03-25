ROS2 for .NET
=============

Build status
------------

| Target | Status |
|----------|--------|
| **Universal Windows Platform (x86/x64/ARM)** | ![Build (UWP)](https://github.com/ros2-dotnet/ros2_dotnet/workflows/Build%20(UWP)/badge.svg) |
| **Windows Desktop (x64)** | ![Build (Desktop)](https://github.com/ros2-dotnet/ros2_dotnet/workflows/Build%20(Desktop)/badge.svg) |
| **Linux** | (TODO) |

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

First of all install the standard ROS2 dependencies for your operating system of choice https://github.com/ros2/ros2/wiki/Installation#building-from-source

Next make sure you've either installed .Net Core (preferred) https://www.microsoft.com/net/learn/get-started or Mono https://www.mono-project.com/

The following steps show how to build the examples on Windows and Linux:

Windows (Desktop)
-----------------

```
md \dev\ros2_dotnet_ws\src
cd \dev\ros2_dotnet_ws
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos -o ros2_dotnet.repos
vcs import \dev\ros2_dotnet_ws\src < ros2_dotnet.repos
colcon build --merge-install
```

Linux
-----

```
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet.repos
colcon build --merge-install
```

Universal Windows Platform (ARM, Win32, Win64)
----------------------------------------------

We'll build this in two steps, first `ament` (the build system) and then ROS2 itself. This is because `ament` will run on the host platform, whereas `ros2-dotnet` on the target.

ament
-----

```
md \dev\ament\src
cd \dev\ament
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ament_dotnet_uwp.repos -o ament_dotnet_uwp.repos
vcs import src < ament_dotnet_uwp.repos
colcon build --merge-install
call install\local_setup.bat
```

UWP
---

Replace `%TARGET_ARCH%` with Win32, x64 or ARM

```
md \dev\ros2\src
cd \dev\ros2
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet_uwp.repos -o ros2_dotnet_uwp.repos
vcs import src < ros2_dotnet_uwp.repos
cd \dev\ament
call install\local_setup.bat
cd \dev\ros2
colcon build --merge-install --packages-ignore rmw_fastrtps_dynamic_cpp rcl_logging_log4cxx --cmake-args -A "%TARGET_ARCH%" -DCMAKE_SYSTEM_NAME=WindowsStore -DCMAKE_SYSTEM_VERSION=10.0.17763 -DTHIRDPARTY=ON -DINSTALL_EXAMPLES=OFF -DBUILD_TESTING=OFF
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
