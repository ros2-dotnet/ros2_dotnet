ROS2 for .NET
=============

| Target | Status |
|----------|--------|
| **Linux** | [![Build (Linux)](https://github.com/ros2-dotnet/ros2_dotnet/actions/workflows/build_linux.yml/badge.svg)](https://github.com/ros2-dotnet/ros2_dotnet/actions/workflows/build_linux.yml) |
| **Windows Desktop** | [![Build (Desktop)](https://github.com/ros2-dotnet/ros2_dotnet/actions/workflows/build_desktop.yml/badge.svg)](https://github.com/ros2-dotnet/ros2_dotnet/actions/workflows/build_desktop.yml) |

_Windows UWP CI Builds are currently disabled, see [this issue](https://github.com/ros2-dotnet/ros2_dotnet/issues/92) for more information._

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for .NET Core and .NET Standard.

Features
--------

The current set of features include:
- Generation of all builtin ROS types
- Support for publishers and subscriptions
- Support for clients and services
- Cross-platform support (Linux, Windows)
  - For using ROS 2 with Hololens see https://github.com/ms-iot/ros_msft_mrtk

What's missing?
---------------

Lots of things!
- Unicode types
- String constants (specifically BoundedString)
- Component nodes
- Actions
- Tests
- Documentation
- More examples

Sounds great, how can I try this out?
-------------------------------------

First of all install the standard ROS2 dependencies for your operating system
of choice: https://github.com/ros2/ros2/wiki/Installation#building-from-source

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
curl -sk https://raw.githubusercontent.com/ros2-dotnet/ros2_dotnet/main/ros2_dotnet_foxy.repos -o ros2_dotnet_foxy.repos
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
wget https://raw.githubusercontent.com/ros2-dotnet/ros2_dotnet/main/ros2_dotnet_foxy.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet_foxy.repos
colcon build
```

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
